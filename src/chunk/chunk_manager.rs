use crate::{data::voxel_octree::{VoxelOctree, MeshData, ParentValueType}, utils::get_chunk_coords};

use super::*;
use hashbrown::HashMap;
use noise::*;
use num_traits::Pow;

#[derive(Default)]
pub struct LoadedChunk {
  pub key: [u32; 3],
  pub ttl: f32,
}

#[derive(Default)]
pub struct SubscribeData {
  pub chunks: HashMap<[u32; 3], VoxelOctree>,
  pub rays: Vec<[f32; 3]>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkMode {
  None,
  Loaded,
  Unloaded,
  Air,
  Inner,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Deployment {
  Production,
  Development,
}

#[derive(Clone, Debug)]
pub struct Chunk {
  pub key: [i64; 3],
  pub octree: VoxelOctree,
  pub mode: ChunkMode,
  pub is_default: bool,
}

impl Default for Chunk {
  fn default() -> Chunk {
    Chunk {
      key: [0, 0, 0],
      octree: VoxelOctree::new(0, 4),
      mode: ChunkMode::Unloaded,
      is_default: true,
    }
  }
}

pub struct ChunkManager {
  pub chunks: HashMap<[i64; 3], Chunk>,
  colliders: HashMap<[i64; 3], Chunk>,
  pub depth: u32,
  pub chunk_size: u32,
  pub offset: u32,
  pub noise: OpenSimplex,
  pub height_scale: f64,
  pub frequency: f64,

  // Probably need to put somewhere server and client can access
  pub lod_dist: Vec<i64>,
  pub ray_dist: i64,
  pub deployment: Deployment,
}

impl Default for ChunkManager {
  fn default() -> Self {
    let depth = 4;

    ChunkManager {
      chunks: HashMap::new(),
      colliders: HashMap::new(),
      depth: depth,
      chunk_size: 2_i32.pow(depth) as u32,
      offset: 2,
      noise: OpenSimplex::new().set_seed(1234),
      height_scale: 16.0,
      frequency: 0.0125,

      lod_dist: vec![4, 12],
      ray_dist: 5,
      deployment: Deployment::Production,
    }
  }
}

impl ChunkManager {
  /* TODO: Remove later */
  pub fn set_voxel1(&mut self, pos: &[i64; 3], voxel: u8) -> Vec<[i64; 3]> {
    let mut keys = Vec::new();
    let seamless_size = self.seamless_size();

    let w_key = world_pos_to_key(pos, seamless_size);
    // let w_key = voxel_pos_to_key(pos, seamless_size);
   

    for x in -1..1 {
      for y in -1..1 {
        for z in -1..1 {
          let key_x = w_key[0] + x;
          let key_y = w_key[1] + y;
          let key_z = w_key[2] + z;
          let key = [key_x, key_y, key_z];

          let chunk_sizei64 = (self.chunk_size - 1) as i64;
          let size = seamless_size as i64;

          let min_x = key_x * size;
          let max_x = min_x + chunk_sizei64;
          let min_y = key_y * size;
          let max_y = min_y + chunk_sizei64;
          let min_z = key_z * size;
          let max_z = min_z + chunk_sizei64;
          if pos[0] >= min_x
            && pos[0] <= max_x
            && pos[1] >= min_y
            && pos[1] <= max_y
            && pos[2] >= min_z
            && pos[2] <= max_z
          {
            let local_x = (pos[0] - min_x) as u32;
            let local_y = (pos[1] - min_y) as u32;
            let local_z = (pos[2] - min_z) as u32;

            if let Some(mut chunk) = self.get_chunk_mut(&key) {
              chunk.octree.set_voxel(local_x, local_y, local_z, voxel);

              //  FIXME: Check performance hit
              chunk.mode = chunk_mode(&chunk.octree);

              // if same_coord_i64(&[-1, 0, -1], &key) {
              //   c.mode = chunk_mode(&c.octree);
              // }
              chunk.is_default = false;
              keys.push(key);
            } else {
              let mut chunk = self.new_chunk3(&key, self.depth as u8);
              chunk.octree.set_voxel(local_x, local_y, local_z, voxel);

              //  FIXME: Check performance hit
              chunk.mode = chunk_mode(&chunk.octree);

              chunk.is_default = false;
              self.set_chunk(&key, &chunk);
              keys.push(key);
            }
          }
        }
      }
    }
    return keys;
  }

  pub fn set_voxel2(&mut self, pos: &[i64; 3], voxel: u8) -> Vec<([i64; 3], Chunk)> {
    let mut chunks = Vec::new();

    let coords = get_chunk_coords(pos, voxel);
    for coord in coords.iter() {
      let key = &coord.key;
      let local = &coord.local;

      if let Some(mut chunk) = self.get_chunk_mut(key) {
        chunk.octree.set_voxel(local[0], local[1], local[2], voxel);
        chunks.push((key.clone(), chunk.clone()));
      } else {
        let mut chunk = self.new_chunk3(&key, self.depth as u8);
        chunk.octree.set_voxel(local[0], local[1], local[2], voxel);
        self.set_chunk(key, &chunk);
        chunks.push((key.clone(), chunk.clone()));
      }
    }
    chunks
  }



  pub fn get_voxel(&self, pos: &[i64; 3]) -> u8 {
    let seamless_size = self.seamless_size();
    let key = voxel_pos_to_key(pos, seamless_size);
    // let key = world_pos_to_key(pos, seamless_size);

    let octree = match self.get_octree(&pos) {
      Some(o) => o,
      // None => { println!("none1"); return 0 },
      None => return 0
    };

    let sizei64 = seamless_size as i64;
    let local_x = pos[0] - (key[0] * sizei64);
    let local_y = pos[1] - (key[1] * sizei64);
    let local_z = pos[2] - (key[2] * sizei64);

    // println!("key1 {:?} local {} {} {}", key, local_x, local_y, local_z);

    octree.get_voxel(local_x as u32, local_y as u32, local_z as u32)
  }

  fn get_octree(&self, pos: &[i64; 3]) -> Option<&VoxelOctree> {
    let seamless_size = self.seamless_size();
    let key = &voxel_pos_to_key(pos, seamless_size);
    // let key = &world_pos_to_key(pos, seamless_size);
    // println!("get_octree() Key {:?}", key);
    let chunk = match self.get_chunk(key) {
      Some(o) => o,
      None => return None,
    };
    Some(&chunk.octree)
  }

  /**
   * Deprecated
   */
  pub fn compute_mesh(
    &self,
    mode: VoxelMode,
    chunks: &HashMap<[u32; 3], Chunk>,
  ) -> HashMap<[u32; 3], MeshData> {
    let mut meshes = HashMap::new();
    let seamless_size = self.seamless_size() as f32;
    for (key, chunk) in chunks.iter() {
      let world_key = region_key_to_world_key(key, self.seamless_size());
      let wk_f32 = [
        world_key[0] as f32 * seamless_size,
        world_key[1] as f32 * seamless_size,
        world_key[2] as f32 * seamless_size,
      ];

      // println!("key {:?} wk_f32 {:?}", key, wk_f32);
      // let result = chunk.octree.compute_mesh(mode, &[0.0, 0.0, 0.0]);
      let result = chunk.octree.compute_mesh(mode, &wk_f32);
      // TODO: Investigate later why there is position but no indices
      if result.indices.len() == 0 {
        // println!("no indices {:?}", key);
        continue;
      }
      meshes.insert(key.clone(), result);
    }
    meshes
  }

  pub fn seamless_size(&self) -> u32 {
    self.chunk_size - self.offset
  }

  /**
    TODO: Deprecate later, in favor of using world coord instead of region coord
          We just have to do coord conversion when it is needed in the future
  */
  pub fn new_chunk(key: &[i64; 3], depth: u8, lod_level: u8, noise: OpenSimplex) -> Chunk {
    let size = 2_i32.pow(depth as u32) as u32;
    // if lod_level > depth {
    //   panic!("lod_level: {} cannot be higher than depth: {}", lod_level, depth);
    // }
    let seamless_size = size - 2;
    let region_key = world_key_to_region_key(key, seamless_size);

    let region_middle_pos = region_middle_pos(seamless_size) as i64;
    let start_x = (region_key[0] * seamless_size) + 0;
    let start_y = (region_key[1] * seamless_size) + 0;
    let start_z = (region_key[2] * seamless_size) + 0;

    let new_octree = VoxelOctree::new(0, depth);
    let mut chunk = Chunk {
      key: key.clone(),
      octree: new_octree,
      mode: ChunkMode::None,
      is_default: true,
    };

    let mut has_air = false;
    let mut has_value = false;
    let mut data = Vec::new();

    let power: u32 = 2;
    let diff = (depth - lod_level);
    let step = diff as usize + 1;

    let start = 0;
    let end = size;
    for octree_x in (start..end).step_by(step) {
      for octree_y in (start..end).step_by(step) {
        for octree_z in (start..end).step_by(step) {
          let x = start_x + octree_x;
          let y = start_y + octree_y;
          let z = start_z + octree_z;
          
          let elevation = noise_elevation(&x, &z, &region_middle_pos, noise);
          let mid_y = y as i64 - region_middle_pos;
          let voxel = if mid_y < elevation { 1 } else { 0 };
          data.push([octree_x, octree_y, octree_z, voxel]);

          /*
            TODO:
              Conditions to determine if Chunk is needed to be rendered and create collider
                Mode:
                  Empty/Air
                  Inner
                  Visible
                Air
                  If all values are 0
                Inner
                  If all values are 1
                Visible
                  ?
          */
          if octree_x >= 1
            && octree_x <= end - 1
            && octree_y >= 1
            && octree_y <= end - 1
            && octree_z >= 1
            && octree_z <= end - 1
          {
            if voxel == 0 {
              has_air = true;
              // println!("{} {} {}", octree_x, octree_y, octree_z);
            }
            if voxel == 1 {
              has_value = true;
              
            }
          }
        }
      }
    }

    chunk.octree = VoxelOctree::new_from_3d_array(0, depth, &data, ParentValueType::Lod);
    // chunk.mode = chunk_mode(&chunk.octree);

    /*
      TODO: Have to update mode detector
    */
    if (!has_air && has_value) || (has_air && !has_value) {
      chunk.mode = ChunkMode::Air;  // Should be renamed as empty
    }
    if has_air && has_value {
      chunk.mode = ChunkMode::Loaded;
    }
    // println!("{} {} {}", has_air, has_value, end - 2);
    chunk
  }

  pub fn new_chunk2(key: &[i64; 3], depth: u32, lod_level: u8, noise: OpenSimplex) -> Chunk {
    ChunkManager::new_chunk(key, depth as u8, lod_level, noise)
  }

  pub fn new_chunk3(&self, key: &[i64; 3], lod_level: u8) -> Chunk {
    ChunkManager::new_chunk(key, self.depth as u8, lod_level, self.noise)
  }

  pub fn chunk_mode(self: &Self, key: &[i64; 3]) -> ChunkMode {
    let chunk = self.chunks.get(key);
    let mut mode = ChunkMode::Unloaded;
    if chunk.is_some() {
      mode = chunk.unwrap().mode;
    }
    mode
  }

  pub fn get_chunk(&self, key: &[i64; 3]) -> Option<&Chunk> {
    /* Later on, implement Spatial Partition or R-trees? */
    self.chunks.get(key)
  }

  pub fn get_chunk_mut(&mut self, key: &[i64; 3]) -> Option<&mut Chunk> {
    /* Later on, implement Spatial Partition or R-trees? */
    self.chunks.get_mut(key)
  }

  pub fn set_chunk(&mut self, key: &[i64; 3], chunk: &Chunk) {
    let c = self.chunks.get(key);
    if c.is_some() {
      if !chunk.is_default {
        self.chunks.insert(key.clone(), chunk.clone());
      }
    } else {
      self.chunks.insert(key.clone(), chunk.clone());
    }
  }

  pub fn remove_chunk(&mut self, key: &[i64; 3]) {
    let chunk_op = self.get_chunk(key);
    if chunk_op.is_some() {
      let chunk = chunk_op.unwrap();
      if chunk.is_default {
        self.chunks.remove(key);
      }
    }
  }

  pub fn len(&self) -> usize {
    self.chunks.len()
  }

  fn get_keys(&self, pos: &[i64; 3]) -> Vec<[i64; 3]> {
    let mut keys = Vec::new();
    let seamless_size = self.seamless_size();

    let w_key = world_pos_to_key(pos, seamless_size);

    for x in -1..2 {
      for y in -1..2 {
        for z in -1..2 {
          let key_x = w_key[0] + x;
          let key_y = w_key[1] + y;
          let key_z = w_key[2] + z;
          let key = [key_x, key_y, key_z];
          keys.push(key);
        }
      }
    }
    return keys;
  }

  // FOR TESTING
  pub fn get_noise(&self, x: f64, z: f64, middle: i64) -> i64 {
    let fx = (x as i64 - middle) as f64 * self.frequency;
    let fz = (z as i64 - middle) as f64 * self.frequency;
    let noise = self.noise.get([fx, fz]);
    let elevation = (noise * self.height_scale) as i64;
    elevation
  }
}

fn get_octree_data(
  pos: &[i64; 3],
  chunk_size: u32,
  seamless_size: u32,
) -> (Vec<[i64; 3]>, Vec<[u32; 3]>) {
  let mut keys = Vec::new();
  let mut local_coords = Vec::new();

  /*
    TODO:
      [0, 0, 0] should return:
        [0, 0, 0]: [0, 0, 0]
        []
  */

  (keys, local_coords)
}

/*
  Limit values to
    x > 14 + offset
    y > 14 + offset
    z > 14 + offset
  Where:
    offset = 2;
*/
fn is_within_scope_test(x: u32, y: u32, z: u32, region_middle_pos: i64) -> bool {
  let target = [28, 27, 18];
  let offset = [1, 1, 1];

  let conv = [
    target[0] + region_middle_pos as u32,
    target[1] + region_middle_pos as u32,
    target[2] + region_middle_pos as u32,
  ];

  let start = [
    conv[0] - offset[0],
    conv[1] - offset[1],
    conv[2] - offset[2],
  ];

  let end = [
    conv[0] + offset[0],
    conv[1] + offset[1],
    conv[2] + offset[2],
  ];

  // println!("scope {} {} {}", x, y, z);
  if x >= start[0] && x <= end[0] &&
  y >= start[1] && y <= end[1] &&
  z >= start[2] && z <= end[2] {
    println!("within scope {} {} {}", x, y, z);
    return true;
  }
  false
}



#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_set_and_get_voxel() -> Result<(), String> {
    let mut chunk_manager = ChunkManager::default();

    let start = -10;
    let end = 10;
    let mut new_value = 0;
    
    for x in start..end {
      for y in start..end {
        for z in start..end {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };
          let pos = &[x, y, z];
          chunk_manager.set_voxel1(pos, new_value);
        }
      }
    }

    new_value = 0;
    for x in start..end {
      for y in start..end {
        for z in start..end {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };
          let expected = new_value;
          let pos = &[x, y, z];
          let result = chunk_manager.get_voxel(pos);

          assert_eq!(result, expected, "at pos: {:?}", (x, y, z));
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_chunk_mode() -> Result<(), String> {
    let chunk_size = 16;
    let mut chunk_manager = ChunkManager::default();
    let chunk = chunk_manager.new_chunk3(&[0, -1, 2], chunk_manager.depth as u8);

    assert_eq!(chunk.mode, ChunkMode::Loaded);

    Ok(())
  }
}
