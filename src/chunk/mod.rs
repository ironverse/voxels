use hashbrown::HashMap;
use noise::NoiseFn;
use noise::OpenSimplex;
use num_traits::Pow;

use crate::data::voxel_octree::MeshData;
use crate::data::voxel_octree::VoxelMode;
use crate::data::voxel_octree::VoxelOctree;

use self::chunk_manager::*;

pub mod chunk_manager;

pub fn adjacent_keys_map(
  key: &[u32; 3],
  range: i64,
  include_current: bool,
) -> HashMap<[u32; 3], bool> {
  let mut keys_around = HashMap::new();
  let start = -range as i64;
  // let start = 0; // FIXME: For testing
  let end = range + 1;

  for iter_x in start..end {
    for iter_y in start..end {
      for iter_z in start..end {
        let chunk_x = key[0] as i64 + iter_x;
        let chunk_y = key[1] as i64 + iter_y;
        let chunk_z = key[2] as i64 + iter_z;

        let chunk_key = [chunk_x as u32, chunk_y as u32, chunk_z as u32];
        if include_current {
          keys_around.insert(chunk_key, true);
        }

        if !include_current && !same_coord(key, &chunk_key) {
          keys_around.insert(chunk_key, true);
        }
      }
    }
  }
  keys_around
}

pub fn adjacent_keys_map2(
  key: &[i64; 3],
  range: i64,
  include_current: bool,
) -> HashMap<[i64; 3], bool> {
  let mut keys_around = HashMap::new();
  let start = -range as i64;
  // let start = 0; // FIXME: For testing
  let end = range + 1;

  for iter_x in start..end {
    for iter_y in start..end {
      for iter_z in start..end {
        let chunk_x = key[0] + iter_x;
        let chunk_y = key[1] + iter_y;
        let chunk_z = key[2] + iter_z;

        let chunk_key = [chunk_x, chunk_y, chunk_z];
        if include_current {
          keys_around.insert(chunk_key, true);
        }

        if !include_current && !same_coord_i64(key, &chunk_key) {
          keys_around.insert(chunk_key, true);
        }
      }
    }
  }
  keys_around
}

pub fn adjacent_keys(key: &[i64; 3], range: i64) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -range;
  // let start = 0; // FIXME: For testing
  let end = range + 1;

  for iter_x in start..end {
    for iter_y in start..end {
      for iter_z in start..end {
        let chunk_x = key[0] + iter_x;
        let chunk_y = key[1] + iter_y;
        let chunk_z = key[2] + iter_z;

        let chunk_key = [chunk_x, chunk_y, chunk_z];
        keys_around.push(chunk_key);
      }
    }
  }
  keys_around
}

pub fn adj_delta_keys(prev_key: &[i64; 3], cur_key: &[i64; 3], range: i64) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -range;
  let end = range + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let c_x = cur_key[0] + x;
        let c_y = cur_key[1] + y;
        let c_z = cur_key[2] + z;

        let diff_x = c_x - prev_key[0];
        let diff_y = c_y - prev_key[1];
        let diff_z = c_z - prev_key[2];

        if diff_x.abs() > range || diff_y.abs() > range || diff_z.abs() > range {
          let key = [c_x, c_y, c_z];
          keys_around.push(key);
        }
      }
    }
  }

  keys_around
}

pub fn in_range_by_chunk(pos1: &[i64; 3], pos2: &[i64; 3], range: i64) -> bool {
  let x = pos1[0] - pos2[0];
  let y = pos1[1] - pos2[1];
  let z = pos1[2] - pos2[2];
  // println!("x {} pos {:?} {:?}", x, pos1, pos2);
  (x.abs() <= range && y.abs() <= range && z.abs() <= range)
}

pub fn adjacent_keys_i64(key: &[i64; 3], range: i64, include_current: bool) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -range;
  // let start = 0; // FIXME: For testing
  let end = range + 1;

  for iter_x in start..end {
    for iter_y in start..end {
      for iter_z in start..end {
        let chunk_x = key[0] + iter_x;
        let chunk_y = key[1] + iter_y;
        let chunk_z = key[2] + iter_z;

        let chunk_key = [chunk_x, chunk_y, chunk_z];
        if include_current {
          keys_around.push(chunk_key);
        }

        if !include_current && !same_coord_i64(key, &chunk_key) {
          keys_around.push(chunk_key);
        }
      }
    }
  }
  keys_around
}

pub fn adjacent_keys2(
  key: &[i64; 3],
  lod: i64,
  range: i64,
  include_current: bool,
) -> Vec<[i64; 4]> {
  let mut keys_around = Vec::new();
  let start = -range;
  // let start = 0; // FIXME: For testing
  let end = range + 1;

  for iter_x in start..end {
    for iter_y in start..end {
      for iter_z in start..end {
        let chunk_x = key[0] + iter_x;
        let chunk_y = key[1] + iter_y;
        let chunk_z = key[2] + iter_z;

        let chunk_key = [chunk_x, chunk_y, chunk_z, lod];
        if include_current {
          keys_around.push(chunk_key);
        }

        let tmp_key = [key[0], key[1], key[2], lod];
        if !include_current && !same_coord2(&tmp_key, &chunk_key) {
          keys_around.push(chunk_key);
        }
      }
    }
  }
  keys_around
}

pub fn adjacent_keys_minmax(key: &[i64; 3], min: i64, max: i64) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -max;
  let end = max + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let chunk_x = key[0] + x;
        let chunk_y = key[1] + y;
        let chunk_z = key[2] + z;
        let c_key = [chunk_x, chunk_y, chunk_z];

        if !in_range2(key, &c_key, min, max) {
          continue;
        }

        keys_around.push([c_key[0], c_key[1], c_key[2]]);
      }
    }
  }
  keys_around
}

pub fn adjacent_keys3(key: &[i64; 3], range: i64) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -range;
  let end = range + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let mut dist = i64::abs(x);
        if i64::abs(y) > dist {
          dist = i64::abs(y)
        }
        if i64::abs(z) > dist {
          dist = i64::abs(z)
        }

        let chunk_x = key[0] + x;
        let chunk_y = key[1] + y;
        let chunk_z = key[2] + z;

        let chunk_key = [chunk_x, chunk_y, chunk_z];
        keys_around.push(chunk_key);
      }
    }
  }
  keys_around
}

pub fn adjacent_keys_by_dist(key: &[i64; 3], range: i64) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -range;
  let end = range + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let chunk_x = key[0] + x;
        let chunk_y = key[1] + y;
        let chunk_z = key[2] + z;

        if !in_range(key, &[chunk_x, chunk_y, chunk_z], range) {
          continue;
        }

        let chunk_key = [chunk_x, chunk_y, chunk_z];
        keys_around.push(chunk_key);
      }
    }
  }
  keys_around
}

pub fn adjacent_keys_min(key: &[i64; 3], range: i64, min: i64, lod: i64) -> Vec<[i64; 4]> {
  let mut keys_around = Vec::new();
  let start = -range;
  let end = range + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let mut dist = i64::abs(x);
        if i64::abs(y) > dist {
          dist = i64::abs(y)
        }
        if i64::abs(z) > dist {
          dist = i64::abs(z)
        }
        if dist < min {
          continue;
        }

        let chunk_x = key[0] + x;
        let chunk_y = key[1] + y;
        let chunk_z = key[2] + z;

        let chunk_key = [chunk_x, chunk_y, chunk_z, lod];
        keys_around.push(chunk_key);
      }
    }
  }
  keys_around
}

pub fn in_range(pos1: &[i64; 3], pos2: &[i64; 3], range: i64) -> bool {
  let mut dist_sqr = 0;
  let r = range;
  for (index, val) in pos1.iter().enumerate() {
    let diff = (val - pos2[index]);
    dist_sqr += diff * diff;
  }
  dist_sqr <= r.pow(2)
}

pub fn in_rangef(pos1: &[i64; 3], pos2: &[i64; 3], range: f32) -> bool {
  let mut dist_sqr = 0.0;
  let r = range as f32;
  for (index, val) in pos1.iter().enumerate() {
    let diff = (val - pos2[index]) as f32;
    dist_sqr += diff * diff;
  }
  dist_sqr <= r.pow(2)
}

pub fn in_range2(pos1: &[i64; 3], pos2: &[i64; 3], min: i64, max: i64) -> bool {
  let mut dist_sqr = 0;
  for (index, val) in pos1.iter().enumerate() {
    let diff = val - pos2[index];
    dist_sqr += diff * diff;
  }
  dist_sqr > min.pow(2) && dist_sqr <= max.pow(2)
}

pub fn in_range2f(pos1: &[i64; 3], pos2: &[i64; 3], min: f32, max: f32) -> bool {
  let mut dist_sqr = 0;
  for (index, val) in pos1.iter().enumerate() {
    let diff = val - pos2[index];
    dist_sqr += (diff * diff);
  }
  let d = dist_sqr as f32;
  d >= min.pow(2) && d <= max.pow(2)
}

pub fn delta_keys(prev_key: &[i64; 3], cur_key: &[i64; 3], range: i64) -> Vec<[i64; 3]> {
  let mut keys_around = Vec::new();
  let start = -range;
  let end = range + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let c_x = cur_key[0] + x;
        let c_y = cur_key[1] + y;
        let c_z = cur_key[2] + z;

        let key = [c_x, c_y, c_z];
        if in_range(cur_key, &key, range) && !in_range(prev_key, &key, range) {
          keys_around.push(key);
        }
      }
    }
  }

  keys_around
}

pub fn delta_keys_minmax(
  prev_key: &[i64; 3],
  cur_key: &[i64; 3],
  min: i64,
  max: i64,
  lod: i64,
) -> Vec<[i64; 4]> {
  let mut keys_around = Vec::new();
  let start = -max;
  let end = max + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let c_x = cur_key[0] + x;
        let c_y = cur_key[1] + y;
        let c_z = cur_key[2] + z;

        let chunk = [c_x, c_y, c_z];
        // Getting outer delta
        if in_range(cur_key, &chunk, max) && !in_range(prev_key, &chunk, max) {
          let chunk_key = [c_x, c_y, c_z, lod];
          keys_around.push(chunk_key);
          continue;
        }

        // Getting inner delta
        if in_range(cur_key, &chunk, max)
          && !in_range(cur_key, &chunk, min)
          && in_range(prev_key, &chunk, min)
        {
          let chunk_key = [c_x, c_y, c_z, lod];
          keys_around.push(chunk_key);
        }
      }
    }
  }

  keys_around
}

/** Deprecated */
pub fn unexplored_keys(
  cur_key: &[i64; 3],
  prev_key: &[i64; 3],
  range: i64,
  lod: i64,
) -> Vec<[i64; 4]> {
  let mut keys_around = Vec::new();
  let start = -range;
  let end = range + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let c_x = cur_key[0] + x;
        let c_y = cur_key[1] + y;
        let c_z = cur_key[2] + z;

        let chunk = [c_x, c_y, c_z];
        if in_range(cur_key, &chunk, range) && !in_range(prev_key, &chunk, range) {
          let chunk_key = [c_x, c_y, c_z, lod];
          keys_around.push(chunk_key);
        }
      }
    }
  }

  keys_around
}

/*
  Deprecate in favor of delta_keys_minmax()
*/
pub fn unexplored_keys2(
  cur_key: &[i64; 3],
  prev_key: &[i64; 3],
  min: i64,
  max: i64,
  lod: i64,
) -> Vec<[i64; 4]> {
  let mut keys_around = Vec::new();
  let start = -max;
  let end = max + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let c_x = cur_key[0] + x;
        let c_y = cur_key[1] + y;
        let c_z = cur_key[2] + z;

        let chunk = [c_x, c_y, c_z];
        if in_range(cur_key, &chunk, max) && !in_range(prev_key, &chunk, min) {
          let chunk_key = [c_x, c_y, c_z, lod];
          keys_around.push(chunk_key);
        }

        // if in_rangef(cur_key, &chunk, max as f32) &&
        // !in_rangef(prev_key, &chunk, min as f32) {
        //   let chunk_key = [c_x, c_y, c_z, lod];
        //   keys_around.push(chunk_key);
        // }

        // if in_range2(cur_key, &chunk, min, max) &&
        // !in_range2(prev_key, &chunk, min, max) {
        //   let chunk_key = [c_x, c_y, c_z, lod];
        //   keys_around.push(chunk_key);
        // }
      }
    }
  }

  keys_around
}

pub fn unexplored_keys2f(
  cur_key: &[i64; 3],
  prev_key: &[i64; 3],
  min: f32,
  max: f32,
  lod: i64,
) -> Vec<[i64; 4]> {
  let mut keys_around = Vec::new();
  let start = -max as i64;
  let end = max as i64 + 1;

  for x in start..end {
    for y in start..end {
      for z in start..end {
        let c_x = cur_key[0] + x;
        let c_y = cur_key[1] + y;
        let c_z = cur_key[2] + z;

        let chunk = [c_x, c_y, c_z];
        if in_range2f(cur_key, &chunk, min, max) && !in_range2f(prev_key, &chunk, min, max) {
          let chunk_key = [c_x, c_y, c_z, lod];
          keys_around.push(chunk_key);
        }
      }
    }
  }

  keys_around
}

// FIXME: Have a checker where the world pos should not be more or less that it should be
pub fn region_key_to_world_key(key: &[u32; 3], seamless_size: u32) -> [i64; 3] {
  let pos = region_key_to_world_pos(key, seamless_size);
  world_pos_to_key(&pos, seamless_size)
}

pub fn region_key_to_world_pos(r_key: &[u32; 3], seamless_size: u32) -> [i64; 3] {
  let region_pos = region_key_to_pos(r_key, seamless_size);
  region_pos_to_world_pos(&region_pos, seamless_size)
}

pub fn region_key_to_pos(key: &[u32; 3], seamless_size: u32) -> [u32; 3] {
  [
    key[0] * seamless_size,
    key[1] * seamless_size,
    key[2] * seamless_size,
  ]
}

pub fn world_pos_to_region_key(pos: &[i64; 3], seamless_size: u32) -> [u32; 3] {
  let r_pos = world_pos_to_region_pos(pos, seamless_size);
  region_pos_to_key(&r_pos, seamless_size)
}

pub fn world_key_to_region_key(key: &[i64; 3], seamless_size: u32) -> [u32; 3] {
  let world_pos = world_key_to_pos(key, seamless_size);

  world_pos_to_region_key(&world_pos, seamless_size)
}

pub fn world_key_to_pos(key: &[i64; 3], seamless_size: u32) -> [i64; 3] {
  [
    key[0] * seamless_size as i64,
    key[1] * seamless_size as i64,
    key[2] * seamless_size as i64,
  ]
}

/* TODO: Might be convert what voxel_pos_to_key() implementation */
pub fn world_pos_to_key(pos: &[i64; 3], seamless_size: u32) -> [i64; 3] {
  let mut x = pos[0];
  let mut y = pos[1];
  let mut z = pos[2];
  let seamless_size_i64 = seamless_size as i64;

  // Between -0.epsilon to -seamless_size..., it should be -1
  if x < 0 {
    x -= seamless_size_i64;
  }
  if y < 0 {
    y -= seamless_size_i64;
  }
  if z < 0 {
    z -= seamless_size_i64;
  }
  [
    x / seamless_size_i64,
    y / seamless_size_i64,
    z / seamless_size_i64,
  ]
}

pub fn world_pos_to_key2(pos: &[i64; 3], seamless_size: u32) -> [i64; 3] {
  let mut x = pos[0];
  let mut y = pos[1];
  let mut z = pos[2];
  let seamless_size_i64 = seamless_size as i64;
  [
    x / seamless_size_i64,
    y / seamless_size_i64,
    z / seamless_size_i64,
  ]
}

pub fn region_pos_to_key(pos: &[u32; 3], seamless_size: u32) -> [u32; 3] {
  [
    pos[0] / seamless_size,
    pos[1] / seamless_size,
    pos[2] / seamless_size,
  ]
}

pub fn region_pos_to_world_key(pos: &[u32; 3], seamless_size: u32) -> [i64; 3] {
  let world_pos = region_pos_to_world_pos(pos, seamless_size);
  world_pos_to_key(&world_pos, seamless_size)
}

pub fn voxel_pos_to_key(pos: &[i64; 3], seamless_size: u32) -> [i64; 3] {
  /*
    TODO:
      Return a key from a pos
      Ex:
        seamless_size = 12


        [0, 0, 0] = [-1, -1, -1]
        [2, 2, 2] = [0, 0, 0]
        [15, 15, 15] = [1, 1, 1]

  */
  let seamless_size_i64 = seamless_size as i64;

  let mid = 2;
  let mut x = pos[0] - mid;
  let mut y = pos[1] - mid;
  let mut z = pos[2] - mid;

  // Between -0.epsilon to -seamless_size..., it should be -1
  if x < 0 {
    x -= seamless_size_i64 - 1;
  }
  if y < 0 {
    y -= seamless_size_i64 - 1;
  }
  if z < 0 {
    z -= seamless_size_i64 - 1;
  }

  [
    x / seamless_size_i64,
    y / seamless_size_i64,
    z / seamless_size_i64,
  ]
}

// TODO: Create a f64 version for detecting the nearest target and new voxel position
pub fn region_pos_to_world_pos(pos: &[u32; 3], seamless_size: u32) -> [i64; 3] {
  let middle = region_middle_pos(seamless_size);
  [
    pos[0] as i64 - middle as i64,
    pos[1] as i64 - middle as i64,
    pos[2] as i64 - middle as i64,
  ]
}

pub fn world_pos_to_region_pos(pos: &[i64; 3], seamless_size: u32) -> [u32; 3] {
  let middle = region_middle_pos(seamless_size);
  [
    (pos[0] + middle as i64) as u32,
    (pos[1] + middle as i64) as u32,
    (pos[2] + middle as i64) as u32,
  ]
}

pub fn region_middle_pos(seamless_size: u32) -> u32 {
  (u32::MAX / 2) - ((u32::MAX / 2) % seamless_size)
}

pub fn same_coord(pos1: &[u32; 3], pos2: &[u32; 3]) -> bool {
  pos1[0] == pos2[0] && pos1[1] == pos2[1] && pos1[2] == pos2[2]
}

pub fn same_coord_i64(pos1: &[i64; 3], pos2: &[i64; 3]) -> bool {
  pos1[0] == pos2[0] && pos1[1] == pos2[1] && pos1[2] == pos2[2]
}

pub fn same_coord2(pos1: &[i64; 4], pos2: &[i64; 4]) -> bool {
  for index in 0..pos1.len() {
    if pos1[index] != pos2[index] {
      return false;
    }
  }
  true
}

pub fn chunk_mode(octree: &VoxelOctree) -> ChunkMode {
  let mut mode = ChunkMode::None;

  let size = octree.get_size();
  // let start = 1;
  // let end = size - 1;

  let start = 1;
  let end = size - 2;

  let mut is_air = false;
  let mut has_value = false;
  for x in start..end {
    for y in start..end {
      for z in start..end {
        let voxel = octree.get_voxel(x, y, z);

        if voxel == 1 {
          has_value = true;
        }
        
        if voxel == 0 {
          is_air = true;
        }
        
      }
    }
  }
  if (!is_air && has_value) || (is_air && !has_value) {
    mode = ChunkMode::Air;  // Should be renamed as empty
  }
  // if has_value && !is_air {
  //   mode = ChunkMode::Inner;
  // }
  if is_air && has_value {
    mode = ChunkMode::Loaded;
  }
  // println!("{} {}", is_air, has_value);
  mode
}

fn noise_elevation(x: &u32, z: &u32, middle: &i64, noise: OpenSimplex) -> i64 {
  let frequency = 0.0125;
  // let frequency = 0.05;
  let height_scale = 16.0;
  let fx = (*x as i64 - middle) as f64 * frequency;
  let fz = (*z as i64 - middle) as f64 * frequency;
  let noise = noise.get([fx, fz]);
  let elevation = (noise * height_scale) as i64;
  elevation
}

fn noise_elevation2(x: &i64, z: &i64, middle: &i64, noise: &OpenSimplex) -> i64 {
  let frequency = 0.0125;
  // let frequency = 0.05;
  let height_scale = 16.0;
  let fx = (x - middle) as f64 * frequency;
  let fz = (z - middle) as f64 * frequency;
  let noise = noise.get([fx, fz]);
  let elevation = (noise * height_scale) as i64;
  elevation
}



/** Compute mesh without setting the position based on the region_key */
pub fn compute_mesh(
  chunks: &HashMap<[i64; 3], Chunk>,
  mode: VoxelMode,
) -> HashMap<[i64; 3], MeshData> {
  let mut meshes = HashMap::new();
  for (key, chunk) in chunks.iter() {
    let result = chunk.octree.compute_mesh2(mode);
    // No rendered mesh, can be either empty or fully set(Inner) chunk
    if result.indices.len() == 0 {
      continue;
    }
    meshes.insert(key.clone(), result);
  }
  meshes
}

pub fn get_dist(pos1: &[i64; 3], pos2: &[i64; 3]) -> f32 {
  let mut dist_sqr = 0;
  for (index, val) in pos1.iter().enumerate() {
    let diff = (val - pos2[index]);
    dist_sqr += diff * diff;
  }
  (dist_sqr as f32).sqrt()
}

#[cfg(test)]
mod tests {
  use super::*;
  use hashbrown::HashMap;
  use num_traits::ToPrimitive;

  #[test]
  fn test_world_pos_to_key() -> Result<(), String> {
    let chunk_size = 12;
    let pos = [0, 0, 0];
    let key = world_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [0, 0, 0]);

    let pos = [1, 11, 12];
    let key = world_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [0, 0, 1]);

    let pos = [24, 36, 48];
    let key = world_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [2, 3, 4]);

    let pos = [-1, -11, -12];
    let key = world_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [-1, -1, -2]);

    let pos = [-24, -36, -48];
    let key = world_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [-3, -4, -5]);
    Ok(())
  }

  #[test]
  fn test_region_pos_to_key() -> Result<(), String> {
    let chunk_size = 12;
    let pos = [2147483640, 2147483640, 2147483640];
    let key = region_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [178956970, 178956970, 178956970]);

    let pos = [2147483641, 2147483651, 2147483652];
    let key = region_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [178956970, 178956970, 178956971]);

    let pos = [2147483639, 2147483629, 2147483628];
    let key = region_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [178956969, 178956969, 178956969]);
    Ok(())
  }

  #[test]
  fn test_region_pos_to_world_pos() -> Result<(), String> {
    let chunk_size = 12;
    let region_pos = [0, 11, 12];
    let world_pos = region_pos_to_world_pos(&region_pos, chunk_size);
    assert_eq!(world_pos, [-2147483640, -2147483629, -2147483628]);

    let region_pos = [2147483639, 2147483640, 2147483641];
    let world_pos = region_pos_to_world_pos(&region_pos, chunk_size);
    assert_eq!(world_pos, [-1, 0, 1]);

    let region_pos = [4294967280, 4294967280, 4294967280];
    let world_pos = region_pos_to_world_pos(&region_pos, chunk_size);
    assert_eq!(world_pos, [2147483640, 2147483640, 2147483640]);

    Ok(())
  }

  #[test]
  fn test_world_pos_to_region_pos() -> Result<(), String> {
    let chunk_size = 12;
    let world_pos = [0, 0, 0];
    let region_pos = world_pos_to_region_pos(&world_pos, chunk_size);
    assert_eq!(region_pos, [2147483640, 2147483640, 2147483640]);

    let world_pos = [-1, -2, -3];
    let region_pos = world_pos_to_region_pos(&world_pos, chunk_size);
    assert_eq!(region_pos, [2147483639, 2147483638, 2147483637]);

    let world_pos = [1, 2, 3];
    let region_pos = world_pos_to_region_pos(&world_pos, chunk_size);
    assert_eq!(region_pos, [2147483641, 2147483642, 2147483643]);
    Ok(())
  }

  #[test]
  fn test_region_pos_to_world_key() -> Result<(), String> {
    let chunk_size = 12;
    let region_pos = [2147483640, 2147483640, 2147483640];
    let world_key = region_pos_to_world_key(&region_pos, chunk_size);
    assert_eq!(world_key, [0, 0, 0]);

    let region_pos = [2147483641, 2147483651, 2147483652];
    let world_key = region_pos_to_world_key(&region_pos, chunk_size);
    assert_eq!(world_key, [0, 0, 1]);

    let region_pos = [2147483639, 2147483629, 2147483628];
    let world_key = region_pos_to_world_key(&region_pos, chunk_size);
    assert_eq!(world_key, [-1, -1, -1]);
    Ok(())
  }

  #[test]
  fn test_world_pos_to_region_key() -> Result<(), String> {
    let chunk_size = 12;
    let world_pos = [0, 0, 0];
    let region_key = world_pos_to_region_key(&world_pos, chunk_size);
    assert_eq!(region_key, [178956970, 178956970, 178956970]);

    let world_pos = [-1, -12, -24];
    let region_key = world_pos_to_region_key(&world_pos, chunk_size);
    assert_eq!(region_key, [178956969, 178956969, 178956968]);

    let world_pos = [1, 12, 24];
    let region_key = world_pos_to_region_key(&world_pos, chunk_size);
    assert_eq!(region_key, [178956970, 178956971, 178956972]);

    let world_pos = [12, 24, 36];
    let region_key = world_pos_to_region_key(&world_pos, chunk_size);
    assert_eq!(region_key, [178956971, 178956972, 178956973]);

    let world_pos = [-12, -24, -36];
    let region_key = world_pos_to_region_key(&world_pos, chunk_size);
    assert_eq!(region_key, [178956969, 178956968, 178956967]);

    Ok(())
  }

  #[test]
  fn test_world_key_to_region_key() -> Result<(), String> {
    let seamless_size = 12;
    let world_key = [0, 0, 0];
    let region_key = world_key_to_region_key(&world_key, seamless_size);
    assert_eq!(region_key, [178956970, 178956970, 178956970]);

    let world_key = [1, 2, 3];
    let region_key = world_key_to_region_key(&world_key, seamless_size);
    assert_eq!(region_key, [178956971, 178956972, 178956973]);

    let world_key = [-1, -2, -3];
    let region_key = world_key_to_region_key(&world_key, seamless_size);
    assert_eq!(region_key, [178956969, 178956968, 178956967]);

    Ok(())
  }

  #[test]
  fn test_voxel_pos_to_key() -> Result<(), String> {
    let chunk_size = 12;

    let pos = [1, 1, 1];
    let key = voxel_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [-1, -1, -1]);

    let pos = [-10, -11, -12];
    let key = voxel_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [-1, -2, -2]);

    let pos = [-22, -23, -24];
    let key = voxel_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [-2, -3, -3]);

    let pos = [2, 2, 2];
    let key = voxel_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [0, 0, 0]);

    let pos = [13, 14, 14];
    let key = voxel_pos_to_key(&pos, chunk_size);
    assert_eq!(key, [0, 1, 1]);
    Ok(())
  }

  // FOR TESTING, HAVE TO CONVERT INTO UNIT TESTS ONCE WORKING PROPERLY

  /* 
    TODO
      Refactor to make it a unit test later when stabilized
  */
  #[test]
  fn test_chunk_mode() -> Result<(), String> {
    let mut chunk_manager = ChunkManager::default();
    // let chunk = chunk_manager.new_chunk3(&[0, 0, 0], chunk_manager.depth as u8);
    // chunk_manager.set_voxel2(&[-1, -1, -1], 1);
    chunk_manager.set_voxel2(&[-1, 0, -1], 1);
    chunk_manager.set_voxel2(&[-1, 1, -1], 1);
    // chunk_manager.set_voxel2(&[-1, 2, -1], 1);

    let mut keys = [
      // [-1,-1,-1],
      [-1, 0,-1]
    ];

    let start = 1;
    let end = 14;

    for (key, chunk) in chunk_manager.chunks.iter() {
      if !keys.contains(key) {
        continue;
      }
      println!("key {:?} {:?}", key, chunk.mode);
      
      for x in start..end {
        println!("x {}", x);
        for y in start..end {
          for z in start..end {
            let voxel = chunk.octree.get_voxel(x, y, z);
            if voxel == 1 {
              println!("{} {} {} {}", x, y, z, voxel);
            }
            
          }
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_chunk_mode_new_chunk() -> Result<(), String> {
    let mut chunk_manager = ChunkManager::default();
    let r_key = world_key_to_region_key(&[-1, 0, -1], chunk_manager.seamless_size());
    let chunk = ChunkManager::new_chunk(
      &r_key, 
      chunk_manager.depth as u8, 
      chunk_manager.depth as u8, 
      chunk_manager.noise
    );

    println!("ChunkMode {:?}", chunk.mode);
    let start = 1;
    let end = 14;

    for x in start..end {
      for y in start..end {
        for z in start..end {
          let voxel = chunk.octree.get_voxel(x, y, z);
          if voxel == 1 {
            println!("{} {} {} {}", x, y, z, voxel);
          }
          
        }
      }
    }

    Ok(())
  }



  #[test]
  fn test_adj_delta_keys() -> Result<(), String> {
    let prev_key = [-1, 0, 0];
    let cur_key = [0, 0, 0];
    let range = 1;

    let keys = adj_delta_keys(&prev_key, &cur_key, range);
    for key in keys.iter() {
      println!("{:?}", key);
    }
    Ok(())
  }

  // #[test]
  // fn test_adjacent_keys_by_dist() -> Result<(), String> {
  //   let max = 5;
  //   let cur_key = [0, 0, 0];
  //   let keys = adjacent_keys_by_dist(&cur_key, max);

  //   for key in keys.iter() {
  //     let key3 = [key[0], key[1], key[2]];
  //     let dist = get_dist(&cur_key, &key3);
  //     if dist as i64 == max {
  //       // println!("{:?} {:?}", key, dist);
  //     }

  //   }

  //   Ok(())
  // }

  // #[test]
  // fn test_adjacent_keys_minmax() -> Result<(), String> {
  //   let min = 5;
  //   let max = 7;
  //   let cur_key = [0, 0, 0];
  //   let keys = adjacent_keys_minmax(&cur_key, min + 1, max);

  //   let keys_dist = adjacent_keys_by_dist(&cur_key, min);

  //   for key in keys.iter() {
  //     let key3 = [key[0], key[1], key[2]];
  //     let dist = get_dist(&cur_key, &key3);

  //     for key_dist in keys_dist.iter() {
  //       let k = [key_dist[0], key_dist[1], key_dist[2]];
  //       if same_coord_i64(&key3, &k) {
  //         println!("{:?} {:?}", k, dist);
  //       }
  //     }

  //     // if dist == min as f32 {
  //       // println!("{:?} {:?}", key, dist);
  //     // }

  //   }

  //   Ok(())
  // }

  // #[test]
  // fn test_adjacent_keys3() -> Result<(), String> {
  //   let cur_key = [0, 0, 0];
  //   let max = 5;
  //   let keys = adjacent_keys3(&cur_key, max);

  //   for key in keys.iter() {
  //     let key3 = [key[0], key[1], key[2]];
  //     let dist = get_dist(&cur_key, &key3);
  //     if dist as i64 >= max {
  //       println!("{:?} {:?}", key, dist);
  //     }
  //   }

  //   Ok(())
  // }

  // #[test]
  // fn test_in_range() -> Result<(), String> {
  //   let range = 5;
  //   let pos1 = [0, 0, 0];
  //   let pos2 = [6, 0, 0];
  //   if in_range(&pos1, &pos2, range) {
  //     // println!("in_range");
  //   }

  //   // let start_key = [0, 0, 0];
  //   // let range = 10;
  //   // let max_lod = 4;
  //   // let keys = adjacent_keys3(&start_key, range, max_lod);

  //   // let dist = 5;
  //   // for key in keys.iter() {
  //   //   let pos = [key[0], key[1], key[2]];
  //   //   if in_range(&start_key, &pos, dist) {
  //   //     for k in key.iter() {
  //   //       let abs = k.abs();
  //   //       assert!(abs <= dist, "k {} should be lower equal than dist {}", abs, dist);
  //   //     }
  //   //   }
  //   // }

  //   Ok(())
  // }

  // #[test]
  // fn test_in_range2() -> Result<(), String> {
  //   let min = 5;
  //   let max = 10;
  //   let pos1 = [0, 0, 0];
  //   let pos2 = [6, 0, 0];
  //   if in_range2(&pos1, &pos2, min, max) {
  //     // println!("in_range");
  //   }
  //   Ok(())
  // }

  // #[test]
  // fn test_in_range2f() -> Result<(), String> {
  //   let min = 5.000001;
  //   let max = 10.0;
  //   let pos1 = [0, 0, 0];
  //   let pos2 = [5, 0, 0];
  //   if in_range2f(&pos1, &pos2, min, max) {
  //     println!("in_range");
  //   }
  //   Ok(())
  // }

  // #[test]
  // fn test_player_adjacent_keys() -> Result<(), String> {
  //   let min = 5;
  //   let max = 10;
  //   let lod = 4;
  //   let prev_key = [-1, 0, 0];
  //   let cur_key = [0, 0, 0];

  //   let keys_lod4 = adjacent_keys_by_dist(&cur_key, min);
  //   let keys_lod3 = adjacent_keys_minmax(&cur_key, min, max);
  //   let mut fusion = keys_lod4.clone();
  //   fusion.extend(keys_lod3.iter());

  //   let start = -max;
  //   let end = max + 1;

  //   // FIXME: Update later

  //   // for x in start..end {
  //   //   for y in start..end {
  //   //     for z in start..end {
  //   //       let mut tmp_lod = lod;
  //   //       let key3 = [x, y, z];
  //   //       let dist = get_dist(&cur_key, &key3);

  //   //       if dist > min as f32 {
  //   //         tmp_lod = lod - 1;
  //   //       }
  //   //       let key4 = [x, y, z, tmp_lod];

  //   //       if same_coord(&[0, -5, 0], &key3) {
  //   //         println!("Testing {} in {:?} : {}", lod, key4, dist)
  //   //       }

  //   //       if dist <= min as f32 {
  //   //         assert_eq!(lod, key4[3], "Lod should be {} in {:?} : {}", lod, key4, dist);
  //   //       }

  //   //       if dist > min as f32 {
  //   //         assert_eq!(lod - 1, key4[3], "Lod should be {} in {:?} : {}", lod, key4, dist);
  //   //       }

  //   //       if dist > max as f32 {
  //   //         assert!(
  //   //           !fusion.contains(&key4),
  //   //           "Exceed max range, should not exists {:?} : {}",
  //   //           key4, dist
  //   //         );
  //   //         continue;
  //   //       }

  //   //       assert!(fusion.contains(&key4), "Should exists {:?} : {}", key4, dist);
  //   //     }
  //   //   }
  //   // }

  //   Ok(())
  // }

  // #[test]
  // fn test_player_adjacent_delta_keys() -> Result<(), String> {
  //   let min = 5;
  //   let max = 10;
  //   let lod = 4;
  //   let prev_key = [-1, 0, 0];
  //   let cur_key = [0, 0, 0];

  //   let adj_keys = adjacent_keys_by_dist(&prev_key, min);
  //   let delta_keys = delta_keys(&prev_key, &cur_key, min);

  //   for adj_key in adj_keys.iter() {
  //     assert!(!delta_keys.contains(adj_key), "Should not exists {:?}", adj_key);
  //   }

  //   let start = -max;
  //   let end = max + 1;

  //   for x in start..end {
  //     for y in start..end {
  //       for z in start..end {
  //         let key3 = [x, y, z];
  //         let dist = get_dist(&cur_key, &key3);
  //         // let key4 = [x, y, z, lod];

  //         if dist > min as f32 {
  //           assert!(!delta_keys.contains(&key3), "Should not exists {:?}", key3);
  //         }
  //       }
  //     }
  //   }
  //   Ok(())
  // }

  // #[test]
  // fn test_player_adjacent_delta_keys_minmax() -> Result<(), String> {
  //   // let min = 5;
  //   // let max = 10;
  //   // let lod = 4;
  //   // let prev_key = [-1, 0, 0];
  //   // let cur_key = [0, 0, 0];

  //   // let d_keys_minmax = delta_keys_minmax(&prev_key, &cur_key, min, max);
  //   // let cur_adj_keys_min = adjacent_keys_by_dist(&cur_key, min);

  //   // // Should not have any keys inside the current min range
  //   // for key in cur_adj_keys_min.iter() {
  //   //   assert!(!d_keys_minmax.contains(key), "Should not exists {:?}", key);
  //   // }
  //   // for key in d_keys_minmax.iter() {
  //   //   assert!(!cur_adj_keys_min.contains(key), "Should not exists {:?}", key);
  //   // }

  //   // let prev_adj_keys_min = adjacent_keys_by_dist(&prev_key, min);
  //   // let prev_adj_keys_max = adjacent_keys_by_dist(&prev_key, max);

  //   // /*
  //   //   Should only have values outside current min range and
  //   //   Inside the previous max range and
  //   //   Inside the previous min range
  //   // */
  //   // for key in prev_adj_keys_max.iter() {

  //   //   if d_keys_minmax.contains(key) {
  //   //     if !cur_adj_keys_min.contains(key) && prev_adj_keys_min.contains(key) {

  //   //     } else {
  //   //       assert!(!d_keys_minmax.contains(key), "Should not exists {:?}", key);
  //   //     }
  //   //   }

  //   // }

  //   // let start = -max;
  //   // let end = max + 1;
  //   // for x in start..end {
  //   //   for y in start..end {
  //   //     for z in start..end {

  //   //       let key3 = [x, y, z];
  //   //       let dist = get_dist(&cur_key, &key3);
  //   //       // let key4 = [x, y, z, lod];

  //   //       // Should not exceed max range
  //   //       if dist > max as f32 {
  //   //         assert!(!d_keys_minmax.contains(&key3), "Should not exists {:?}", key3);
  //   //       }

  //   //     }
  //   //   }
  //   // }

  //   Ok(())
  // }

  // // FIXME: Consider fixing later
  // #[test]
  // fn test_unexplored_keys_vs_unexplored_keys2() -> Result<(), String> {
  //   let min = 5;
  //   let max = 10;
  //   let prev_key = [-1, 0, 0];
  //   let cur_key = [0, 0, 0];

  //   let keys_lod4 = adjacent_keys_by_dist(&cur_key, min);
  //   let keys_lod3 = adjacent_keys_minmax(&cur_key, min, max);
  //   let mut fusion = keys_lod4.clone();
  //   fusion.extend(keys_lod3.iter());

  //   // let delta_keys_lod4 = unexplored_keys(&cur_key, &prev_key, min);
  //   // let delta_keys_lod3 = unexplored_keys2(&cur_key, &prev_key, min, max);

  //   // let mut delta_fusion = delta_keys_lod4.clone();
  //   // delta_fusion.extend(delta_keys_lod3.iter());

  //   // for existing_key in fusion.iter() {
  //   //   assert!(!delta_fusion.contains(existing_key), "Duplicate {:?}", existing_key);
  //   // }

  //   /*
  //     TODO
  //       Monitor for duplicates
  //       Currently working on missing keys
  //   */
  //   // for x in -max..max + 1 {
  //   //   for y in -max..max + 1 {
  //   //     for z in -max..max + 1 {
  //   //       let mut tmp_lod = lod;

  //   //       let key3 = [x, y, z];
  //   //       let dist = get_dist(&cur_key, &key3);
  //   //       if dist > max as f32 {
  //   //         continue;
  //   //       }

  //   //       if dist > min as f32 {
  //   //         tmp_lod = lod - 1;
  //   //       }

  //   //       let key = [x, y, z, tmp_lod];
  //   //       if !merge.contains(&key) {
  //   //         println!("Should have {:?} {:?}", key, dist);
  //   //       }
  //   //     }
  //   //   }
  //   // }

  //   Ok(())
  // }

  // #[test]
  // fn test_unexplored_keys2() -> Result<(), String> {
  //   // let min = 5;
  //   // let max = 7;
  //   // let lod = 4;
  //   // let prev_key = [-1, 0, 0];
  //   // let cur_key = [0, 0, 0];
  //   // let keys = unexplored_keys2(&cur_key, &prev_key, min, max, lod);

  //   // for key in keys.iter() {
  //   //   let key3 = [key[0], key[1], key[2]];
  //   //   let dist = get_dist(&cur_key, &key3);
  //   //   if dist as i64 >= min && dist as i64 <= min + 1 {
  //   //     println!("{:?} {:?}", key, dist);
  //   //   }
  //   // }
  //   Ok(())
  // }

  // #[test]
  // fn test_remove_keys() -> Result<(), String> {
  //   let start_key = [0, 0, 0];
  //   let range = 10;
  //   let keys = adjacent_keys3(&start_key, range);

  //   use std::time::Instant;
  //   let now = Instant::now();

  //   // let mut keys_clone = keys.clone();
  //   // for key in keys.iter() {
  //   //   let index = keys_clone
  //   //     .iter()
  //   //     .position(|&k| same_coord2(&k, key))
  //   //     .unwrap();
  //   //   keys_clone.swap_remove(index);
  //   // }

  //   // let mut keys_map = HashMap::new();
  //   // for k in keys.iter() {
  //   //   keys_map.insert(k, true);
  //   // }

  //   // for k in keys.iter() {
  //   //   keys_map.remove(k);
  //   // }
  //   // println!("keys_map.len() {}", keys_map.len());

  //   let elapsed = now.elapsed();
  //   println!("Elapsed: {:.2?}", elapsed);

  //   Ok(())
  // }

  // #[test]
  // fn test_pop_keys() -> Result<(), String> {
  //   let start_key = [0, 0, 0];
  //   let range = 10;
  //   let lod = 4;
  //   let keys = adjacent_keys3(&start_key, range);

  //   use std::time::Instant;
  //   let now = Instant::now();

  //   // let mut keys_clone = keys.clone();
  //   // for key in keys.iter().rev() {
  //   //   keys_clone.pop();
  //   // }

  //   // let mut keys_map = HashMap::new();
  //   // for k in keys.iter() {
  //   //   keys_map.insert(k, true);
  //   // }

  //   // for k in keys.iter() {
  //   //   keys_map.remove(k);
  //   // }
  //   // println!("keys_map.len() {}", keys_map.len());

  //   let elapsed = now.elapsed();
  //   println!("Elapsed: {:.2?}", elapsed);

  //   Ok(())
  // }

  // #[test]
  // fn test_pop_keys2() -> Result<(), String> {
  //   let mut keys = vec![
  //     [0, 0, 0],
  //     [1, 0, 0],
  //     [2, 0, 0],
  //     [3, 0, 0],
  //   ];

  //   let mut keys_map = HashMap::new();
  //   for key in keys.iter() {
  //     if key[0] == 1 {
  //       continue;
  //     }
  //     keys_map.insert(key.clone(), true);
  //   }

  //   let clone = keys.clone();
  //   for key in clone.iter().rev() {
  //     if keys_map.contains_key(key) {
  //       keys.pop();
  //       continue;
  //     }
  //     break;
  //   }

  //   // println!("keys {:?}", keys);

  //   Ok(())
  // }
}
