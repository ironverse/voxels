pub mod grid_hashmap;

use hashbrown::HashMap;
use parry3d::math::Point;
use crate::data::CUBE_EDGES;
use crate::chunk::{world_pos_to_key2, voxel_pos_to_key};
use crate::data::surface_nets::estimate_surface_edge_intersection;
use crate::data::voxel_octree::VoxelOctree;

pub struct Utils;

impl Utils {
  pub fn create_z_faces(right_index: u32, right_bottom_index: u32, bottom_index: u32) -> bool {
    right_index != std::u32::MAX
      && right_bottom_index != std::u32::MAX
      && bottom_index != std::u32::MAX
  }
  
  pub fn create_x_faces(back_index: u32, back_bottom_index: u32, bottom_index: u32) -> bool {
    back_index != std::u32::MAX && 
    back_bottom_index != std::u32::MAX && 
    bottom_index != std::u32::MAX
  }
  
  pub fn create_y_faces(right_back_index: u32, right_index: u32, back_index: u32) -> bool {
    right_back_index != std::u32::MAX && right_index != std::u32::MAX && back_index != std::u32::MAX
  }

  pub fn has_pos(cur_pos: &[f32; 3], pos: &[f32; 3], dir: &[i32; 3]) -> bool {
    let x = pos[0] as i32;
    let y = pos[1] as i32;
    let z = pos[2] as i32;
    let cur_x = cur_pos[0] as i32 + dir[0];
    let cur_y = cur_pos[1] as i32 + dir[1];
    let cur_z = cur_pos[2] as i32 + dir[2];
    cur_x == x && cur_y == y && cur_z == z
  }
  
  /* DEPRECATE LATER */
  pub fn has_voxel(octree: &VoxelOctree, pos: &[u32; 3], dir: &[u32; 3]) -> bool {
    octree.get_voxel(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]) > 0
  }
  
  pub fn has_voxel2(voxels: &Vec<u8>, start: u32, end: u32, pos: &[u32; 3], dir: &[u32; 3]) -> bool {
    // octree.get_voxel(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]) > 0
    let x = pos[0] + dir[0];
    let y = pos[1] + dir[1];
    let z = pos[2] + dir[2];
    let index = coord_to_index(x, y, z, start, end);
    if index >= voxels.len() {
      return false;
    }
    voxels[index] > 0
  }
}



pub fn posf32_to_world_key(pos: &[f32; 3], seamless_size: u32) -> [i64; 3] {
  let posi64 = [pos[0] as i64, pos[1] as i64, pos[2] as i64];
  // world_pos_to_key(&posi64, seamless_size)
  voxel_pos_to_key(&posi64, seamless_size)
}



#[derive(Debug)]
pub struct ChunkCoordinate {
  pub key: [i64; 3],
  pub local: [u32; 3]
}


/* 
pub fn create_collider_mesh(octree: &VoxelOctree) -> MeshColliderData {
  let mut possible_positions = Vec::new();
  let mut positions = Vec::new();
  let mut indices = Vec::new();

  let voxel_start = 0;
  let voxel_end = octree.get_size();

  let mut voxels = Vec::new();
  for x in voxel_start..voxel_end {
    for y in voxel_start..voxel_end {
      for z in voxel_start..voxel_end {
        let voxel = octree.get_voxel(x, y, z);
        voxels.push(voxel);
      }
    }
  }

  let start = 1;
  let end = octree.get_size() - 1;

  // Checking for each grid
  for x in start..end {
    for y in start..end {
      for z in start..end {
        // Process the defined voxels first
        // Then do the identification for the voxels via octree later

        // println!("pos {}, {}, {}", x, y, z);
        let (pos, nor) = get_average_vertex_pos(&voxels, voxel_start, voxel_end, x, y, z);
        // let (pos, nor) = get_average_vertex_pos2(octree, x, y, z);
        if pos[0] == 0.0 {
          continue;
        }
        // println!("after pos {}, {}, {}", x, y, z);
        possible_positions.push(pos);

        let mut indices1 = get_indices(
          &voxels,
          voxel_start,
          voxel_end,
          x as i32,
          y as i32,
          z as i32,
          pos,
          &possible_positions,
        );
        
        positions.push(Point::new(pos[0], pos[1], pos[2]));
        // normals.push(nor);
        // uvs.push([0.0, 0.0]);
        indices.append(&mut indices1);
      }
    }
  }
  MeshColliderData {
    positions: positions,
    // normals: normals,
    // uvs: uvs,
    indices: indices,
  }
}
*/

fn get_average_vertex_pos(
  voxels: &Vec<u8>,
  start: u32,
  end: u32,
  x: u32,
  y: u32,
  z: u32,
) -> ([f32; 3], [f32; 3]) {
  let mut avg_pos = [0.0; 3];
  let mut voxel_count = 0;
  let mut dists = [1.0; 8];
  // Checking all corners for voxel values
  for x_offset in 0..2 {
    for y_offset in 0..2 {
      for z_offset in 0..2 {
        let corner_x = x_offset + x;
        let corner_y = y_offset + y;
        let corner_z = z_offset + z;

        let index = coord_to_index(corner_x, corner_y, corner_z, start, end);
        if index >= voxels.len() {
          continue;
        }
        let voxel = voxels[index];
        if voxel > 0 {
          voxel_count += 1;
          let x_index = x_offset;
          let y_index = y_offset << 1;
          let z_index = z_offset << 2;
          let corner_index = x_index as usize + y_index as usize + z_index as usize;
          // println!("x y z {} {} {}", x_index, y_index, z_index);
          dists[corner_index] = -1.0;
        }
      }
    }
  }

  if voxel_count > 0 && voxel_count < 8 {
    let mut count = 0;
    let mut sum = [0.0, 0.0, 0.0];
    for (offset1, offset2) in CUBE_EDGES.iter() {
      if let Some(intersection) =
        estimate_surface_edge_intersection(*offset1, *offset2, dists[*offset1], dists[*offset2])
      {
        count += 1;
        sum[0] += intersection[0];
        sum[1] += intersection[1];
        sum[2] += intersection[2];
        // println!("---",);
        // println!("intersection {:?}", intersection);
      }
    }
    avg_pos[0] = sum[0] / count as f32 + x as f32 + 0.0;
    avg_pos[1] = sum[1] / count as f32 + y as f32 + 0.0;
    avg_pos[2] = sum[2] / count as f32 + z as f32 + 0.0;
    // println!("avg_pos {:?}", avg_pos);
  }

  // What is this?
  // Adding all the axis related dist minus the
  let normal_x = (dists[0b001] + dists[0b011] + dists[0b101] + dists[0b111])
    - (dists[0b000] + dists[0b010] + dists[0b100] + dists[0b110]);
  let normal_y = (dists[0b010] + dists[0b011] + dists[0b110] + dists[0b111])
    - (dists[0b000] + dists[0b001] + dists[0b100] + dists[0b101]);
  let normal_z = (dists[0b100] + dists[0b101] + dists[0b110] + dists[0b111])
    - (dists[0b000] + dists[0b001] + dists[0b010] + dists[0b011]);

  return (avg_pos, [normal_x, normal_y, normal_z]);
}

fn get_indices(
  voxels: &Vec<u8>,
  start: u32,
  end: u32,
  x: i32,
  y: i32,
  z: i32,
  current_pos: [f32; 3],
  possible_positions: &Vec<[f32; 3]>,
) -> Vec<[u32; 3]> {
  // TODO: Revise documentation
  // Check for three faces, start from x0 > right, z0 > z1, y0 > y1
  // See if there is a vertex there

  // Implementation creating faces
  // Connecting positions
  // Have to consider voxel values
  // Starting point: Current Ex: 1, 1, 1
  // 1. Check the current if has position: currently function is called when there is pos
  // 2. Check the bottom y-1
  // 3. Check the z-1
  // 4. Check the bottom z-1, y-1
  // 5. Check the x-1
  // 6. Check the bottom x-1, y-1

  // Have to create a simple guide how the code works
  // So that can be simplified, improve later on
  // Going through grid one by one
  // x0 = current x
  // right = x - 1
  // y0 = current y
  // y1 = y - 1
  // z0 = current z
  // z1 = z - 1

  let mut indices = Vec::new();
  let current = [0, 0, 0];
  let bottom = [0, -1, 0];
  let right = [-1, 0, 0];
  let right_bottom = [-1, -1, 0];
  let back = [0, 0, -1];
  let back_bottom = [0, -1, -1];
  let right_back = [-1, 0, -1];

  let mut current_index = std::u32::MAX;
  let mut bottom_index = std::u32::MAX;
  let mut right_index = std::u32::MAX;
  let mut right_bottom_index = std::u32::MAX;
  let mut back_index = std::u32::MAX;
  let mut back_bottom_index = std::u32::MAX;
  let mut right_back_index = std::u32::MAX;

  // Checking all 8 grid around the voxel
  let mut index = 0;
  for pos in possible_positions {
    if Utils::has_pos(&current_pos, pos, &current) {
      current_index = index;
    }
    if Utils::has_pos(&current_pos, pos, &bottom) {
      bottom_index = index;
    }

    if Utils::has_pos(&current_pos, pos, &right) {
      right_index = index;
    }
    if Utils::has_pos(&current_pos, pos, &right_bottom) {
      right_bottom_index = index;
    }

    if Utils::has_pos(&current_pos, pos, &back) {
      back_index = index;
    }
    if Utils::has_pos(&current_pos, pos, &back_bottom) {
      back_bottom_index = index;
    }

    if Utils::has_pos(&current_pos, pos, &right_back) {
      right_back_index = index;
    }
    index += 1;
  }

  //  Right hand rule
  let mut corner_000 = false; // Right bottom back
  let mut corner_001 = false; // Right bottom front
  let mut corner_100 = false; // Left bottom back
  let mut corner_010 = false; // Right top back

  let cur_pos_u32 = [x as u32, y as u32, z as u32];
  if Utils::has_voxel2(&voxels, start, end, &cur_pos_u32, &[0, 0, 1]) {
    corner_001 = true;
  }
  if Utils::has_voxel2(&voxels, start, end, &cur_pos_u32, &[0, 0, 0]) {
    corner_000 = true;
  }
  if Utils::has_voxel2(&voxels, start, end, &cur_pos_u32, &[1, 0, 0]) {
    corner_100 = true;
  }
  if Utils::has_voxel2(&voxels, start, end, &cur_pos_u32, &[0, 1, 0]) {
    corner_010 = true;
  }

  //  There should be a position in the grid
  //  Either right bottom back or front
  if Utils::create_z_faces(right_index, right_bottom_index, bottom_index) && corner_000 ^ corner_001 {
    //  Normal towards positive z
    //  Else: negative z
    if corner_001 {
      indices.push([current_index, bottom_index, right_index]);
      indices.push([right_index, bottom_index, right_bottom_index]);
    } else {
      indices.push([current_index, right_index, bottom_index]);
      indices.push([right_index, right_bottom_index, bottom_index]);
    }
  }

  if Utils::create_x_faces(back_index, back_bottom_index, bottom_index) && corner_000 ^ corner_100 {
    if corner_000 {
      indices.push([current_index, bottom_index, back_index]);
      indices.push([back_index, bottom_index, back_bottom_index]);
    } else {
      indices.push([current_index, back_index, bottom_index]);
      indices.push([back_index, back_bottom_index, bottom_index]);
    }
  }

  if Utils::create_y_faces(right_back_index, right_index, back_index) && corner_000 ^ corner_010 {
    if corner_000 {
      indices.push([current_index, back_index, right_index]);
      indices.push([right_index, back_index, right_back_index]);
    } else {
      indices.push([current_index, right_index, back_index]);
      indices.push([right_index, right_back_index, back_index]);
    }
  }

  indices
}



fn get_average_vertex_pos2(octree: &VoxelOctree, x: u32, y: u32, z: u32) -> ([f32; 3], [f32; 3]) {
  let mut avg_pos = [0.0; 3];
  let mut voxel_count = 0;
  let mut dists = [1.0; 8];
  // Checking all corners for voxel values
  for x_offset in 0..2 {
    for y_offset in 0..2 {
      for z_offset in 0..2 {
        let corner_x = x_offset + x;
        let corner_y = y_offset + y;
        let corner_z = z_offset + z;
        let voxel = octree.get_voxel(corner_x, corner_y, corner_z);

        // FIXME: Have to modify the condition later
        if voxel > 0 {
          voxel_count += 1;
          let x_index = x_offset;
          let y_index = y_offset << 1;
          let z_index = z_offset << 2;
          let corner_index = x_index as usize + y_index as usize + z_index as usize;
          // println!("x y z {} {} {}", x_index, y_index, z_index);
          dists[corner_index] = -1.0;
        }
      }
    }
  }

  if voxel_count > 0 && voxel_count < 8 {
    let mut count = 0;
    let mut sum = [0.0, 0.0, 0.0];
    for (offset1, offset2) in CUBE_EDGES.iter() {
      if let Some(intersection) =
        estimate_surface_edge_intersection(*offset1, *offset2, dists[*offset1], dists[*offset2])
      {
        count += 1;
        sum[0] += intersection[0];
        sum[1] += intersection[1];
        sum[2] += intersection[2];
        // println!("---",);
        // println!("intersection {:?}", intersection);
      }
    }
    avg_pos[0] = sum[0] / count as f32 + x as f32 + 0.0;
    avg_pos[1] = sum[1] / count as f32 + y as f32 + 0.0;
    avg_pos[2] = sum[2] / count as f32 + z as f32 + 0.0;
    // println!("avg_pos {:?}", avg_pos);
  }

  // What is this?
  // Adding all the axis related dist minus the
  let normal_x = (dists[0b001] + dists[0b011] + dists[0b101] + dists[0b111])
    - (dists[0b000] + dists[0b010] + dists[0b100] + dists[0b110]);
  let normal_y = (dists[0b010] + dists[0b011] + dists[0b110] + dists[0b111])
    - (dists[0b000] + dists[0b001] + dists[0b100] + dists[0b101]);
  let normal_z = (dists[0b100] + dists[0b101] + dists[0b110] + dists[0b111])
    - (dists[0b000] + dists[0b001] + dists[0b010] + dists[0b011]);

  return (avg_pos, [normal_x, normal_y, normal_z]);
}


pub fn coord_to_index(x: u32, y: u32, z: u32, start: u32, end: u32) -> usize {
  let diff = end - start;

  let start_x = x - start;
  let start_y = y - start;
  let start_z = z - start;

  let val_x = diff.pow(2) * start_x;
  let val_y = diff * start_y;
  let val_z = start_z;

  let res = val_x + val_y + val_z;
  res as usize
}

pub fn get_length(depth: u8) -> usize {
  let size = 8;
  let mut len = 0;
  for d in 1..depth {
    len += 8_i32.pow(d as u32);
  }
  len as usize
}


pub fn get_chunk_coords(pos: &[i64; 3], voxel: u8) -> Vec<ChunkCoordinate> {
  /*
    TODO
      Convert world coord to:
        World chunk keys
          Have to identify 
        Local chunk coord
  */


  /*
    TODO
      Create a function where it returns all the converted values
      Or pass the function where to pass the converted values
        To prevent unnecessary allocation of values
      Just do it approach
        Refine the algorithm later
        Has to be reproducible
        And readable to refactor it faster later on

    Current
      Make it work for now
      Stabilize it later
  */

  let chunk_size = 16;
  let seamless_size = chunk_size - 2;

  let mut coords = Vec::new();

  let keys = &potential_keys(pos, seamless_size);
  for key in keys.iter() {
    // println!("key {:?}", key);
    if has_local_coord(pos, key, chunk_size, seamless_size as i64) {
      let local = get_local_coord(pos, key, chunk_size);

      let coord = ChunkCoordinate {
        key: key.clone(),
        local: local
      };
      coords.push(coord);
      // println!("get_chunk_coords key {:?} coord {:?}", key, local);
    }

  }

  coords
}

pub fn potential_keys(pos: &[i64; 3], seamless_size: u32) -> Vec<[i64; 3]> {
  let mut keys = Vec::new();

  let start_key = world_pos_to_key2(pos, seamless_size);
  for x in -1..1 {
    for y in -1..1 {
      for z in -1..1 {
        let key_x = start_key[0] + x;
        let key_y = start_key[1] + y;
        let key_z = start_key[2] + z;
        let key = [key_x, key_y, key_z];
        keys.push(key);
      }
    }
  }
  keys
}

pub fn has_local_coord(
  pos: &[i64; 3], 
  potential_key: &[i64; 3], 
  chunk_size: u32,
  seamless_size: i64
) -> bool {
  let min_x = potential_key[0] * seamless_size;
  let max_x = min_x + chunk_size as i64;
  let min_y = potential_key[1] * seamless_size;
  let max_y = min_y + chunk_size as i64;
  let min_z = potential_key[2] * seamless_size;
  let max_z = min_z + chunk_size as i64;

  // println!("min pos {:?} {:?} {} {}", pos, potential_key, min_x, max_x);

  // FIXME: Refactor later
  if (pos[0] >= min_x && pos[0] < max_x)
    && (pos[1] >= min_y && pos[1] < max_y)
    && (pos[2] >= min_z && pos[2] < max_z)
  {
    return true;
  }
  false
}

pub fn get_local_coord(
  pos: &[i64; 3],
  potential_key: &[i64; 3], 
  chunk_size: u32
) -> [u32; 3] {
  let partitioned_size = chunk_size as i64 - 2;

  let min_x = potential_key[0] * partitioned_size;
  let min_y = potential_key[1] * partitioned_size;
  let min_z = potential_key[2] * partitioned_size;

  /* 
    TODO
      Address the negative position later
  */
  let diff_x = pos[0] - min_x;
  let diff_y = pos[1] - min_y;
  let diff_z = pos[2] - min_z;

  let local_x = diff_x;
  let local_y = diff_y;
  let local_z = diff_z;

  // println!("local {} {} {}", local_x, local_y, local_z);
  [local_x as u32, local_y as u32, local_z as u32]
}


pub fn world_pos_to_octree_coord(pos: &[i64; 3], seamless_size: u32) -> OctreeCoord {
  let key = world_pos_to_octree_key(pos, seamless_size);

  let mut mx = pos[0] % seamless_size as i64;
  let mut my = pos[1] % seamless_size as i64;
  let mut mz = pos[2] % seamless_size as i64;
  if pos[0] < 0 {
    if mx != 0 {
      mx += seamless_size as i64;
    }
    
  }
  if pos[1] < 0 {
    if my != 0 {
      my += seamless_size as i64;
    }
  }
  if pos[2] < 0 {
    if mz != 0 {
      mz += seamless_size as i64;
    }
    
  }
  let local = [mx as u32, my as u32, mz as u32];
  OctreeCoord { key: key, local: local }
}

pub fn world_pos_to_octree_key(pos: &[i64; 3], seamless_size: u32) -> [i64; 3] {
  let mut x = pos[0];
  let mut y = pos[1];
  let mut z = pos[2];
  let seamless_size_i64 = seamless_size as i64;

  /*
    14  to  27 =  1
    0   to  13 =  0
    -1  to -14 = -1
    -15 to -27 = -2
  */

  /*
    if negative:
      num = num + 1
      key = (num / seamless_size) + 1
  */
  // Between -0.epsilon to -seamless_size..., it should be -1
  let mut kx = x / seamless_size_i64;
  let mut ky = y / seamless_size_i64;
  let mut kz = z / seamless_size_i64;
  if x < 0 {
    x += 1;
    kx = (x / seamless_size_i64) - 1;
  }
  if y < 0 {
    y += 1;
    ky = (y / seamless_size_i64) - 1;
  }
  if z < 0 {
    z += 1;
    kz = (z / seamless_size_i64) - 1;
  }
  [kx, ky, kz]
}


pub struct OctreeCoord {
  pub key: [i64; 3],
  pub local: [u32; 3]
}


#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_coord_to_index1() -> Result<(), String> {
    let default_value = 0;
    let mut octree = VoxelOctree::new(default_value, 4);

    let start = 1;
    let end = octree.get_size() - 2;

    // println!("start: end: {} {}",start, end);
    let mut index = 0;
    for x in start..end {
      for y in start..end {
        for z in start..end {
          // println!("right {} {} {} {}", x, y, z, index);
          assert_eq!(
            index,
            coord_to_index(x, y, z, start, end),
            "pos {} {} {}",
            x,
            y,
            z
          );
          index += 1;
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_coord_to_index2() -> Result<(), String> {
    let default_value = 0;
    let mut octree = VoxelOctree::new(default_value, 4);

    let start = 0;
    let end = octree.get_size() - 1;

    // println!("start: end: {} {}",start, end);
    let mut index = 0;
    for x in start..end {
      for y in start..end {
        for z in start..end {
          // println!("right {} {} {} {}", x, y, z, index);
          assert_eq!(
            index,
            coord_to_index(x, y, z, start, end),
            "pos {} {} {}",
            x,
            y,
            z
          );
          index += 1;
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_local_coord() -> Result<(), String> {
    let size = 16;
    let seamless_size = size - 2;
    
    let positions = [
      [ 14, 27, 28],
      [ 0, 13, 0],
      [-1,-14,-15],
      [-28,-29,-42],
    ];
    let expected = [
      OctreeCoord { key: [1, 1, 2], local: [0, 13, 0]},
      OctreeCoord { key: [0, 0, 0], local: [0, 13, 0]},
      OctreeCoord { key: [-1, -1, -2], local: [13, 0, 13]},
      OctreeCoord { key: [-2, -3, -3], local: [0, 13, 0]},
    ];
    for (i, pos) in positions.iter().enumerate() {
      let result = world_pos_to_octree_coord(pos, seamless_size);
      assert_eq!(result.key, expected[i].key, "Wrong key at index {}", i);
      assert_eq!(result.local, expected[i].local, "Wrong local at index {}", i);
    }

    Ok(())
  }


}
