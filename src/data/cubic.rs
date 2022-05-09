use crate::utils::Utils;

use super::voxel_octree::MeshData;
use super::voxel_octree::*;

pub fn get_cube(octree: &VoxelOctree, start_pos: &[f32; 3]) -> MeshData {
  let mut possible_positions = Vec::new();
  let mut positions = Vec::new();
  let mut normals = Vec::new();
  let mut uvs = Vec::new();
  let mut indices = Vec::new();

  // Checking for each grid
  for x in 0..octree.get_size() {
    for y in 0..octree.get_size() {
      for z in 0..octree.get_size() {
        // Process the defined voxels first
        // Then do the identification for the voxels via octree later

        // println!("pos {}, {}, {}", x, y, z);
        // let (pos) = get_pos(octree, x, y, z);
        let pos = get_pos(octree, x, y, z);
        if pos[0] > -std::f32::EPSILON && pos[0] < std::f32::EPSILON {
          continue;
        }
        possible_positions.push(pos);

        let (positions1, mut normals1, mut indices1, mut uvs1) = cubic_indices(
          octree,
          x as i32,
          y as i32,
          z as i32,
          pos,
          &possible_positions,
          &positions,
        );
        // positions.append(&mut positions1);
        // positions.push([pos[0] + start_pos[0], pos[1] + start_pos[1], pos[2] + start_pos[2]]);
        for p in positions1.iter() {
          positions.push([
            p[0] + start_pos[0],
            p[1] + start_pos[1],
            p[2] + start_pos[2],
          ]);
        }
        normals.append(&mut normals1);
        indices.append(&mut indices1);
        uvs.append(&mut uvs1);
        // println!("pos {}, {}, {}", x, y, z);
      }
    }
  }
  MeshData {
    positions: positions,
    normals: normals,
    uvs: uvs,
    indices: indices,
  }
}

fn get_pos(octree: &VoxelOctree, x: u32, y: u32, z: u32) -> [f32; 3] {
  let mut avg_pos = [0.0; 3];
  let mut voxel_count = 0;
  // Checking all edges for voxel values
  for x_offset in 0..2 {
    for y_offset in 0..2 {
      for z_offset in 0..2 {
        let edge_x = x_offset + x;
        let edge_y = y_offset + y;
        let edge_z = z_offset + z;
        let voxel = octree.get_voxel(edge_x, edge_y, edge_z);
        if voxel > 0 {
          voxel_count += 1;
        }
      }
    }
  }

  if voxel_count > 0 && voxel_count < 8 {
    avg_pos[0] = x as f32 + 0.5;
    avg_pos[1] = y as f32 + 0.5;
    avg_pos[2] = z as f32 + 0.5;
  }
  return avg_pos;
}

fn cubic_indices(
  octree: &VoxelOctree,
  x: i32,
  y: i32,
  z: i32,
  current_pos: [f32; 3],
  possible_positions: &Vec<[f32; 3]>,
  positions: &Vec<[f32; 3]>,
) -> (Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<u32>, Vec<[f32; 2]>) {
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

  let mut tmp_positions = Vec::new();
  let mut indices = Vec::new();
  let mut normals = Vec::new();
  let mut uvs = Vec::new();
  let bottom = [0, -1, 0];
  let right = [-1, 0, 0];
  let right_bottom = [-1, -1, 0];
  let back = [0, 0, -1];
  let back_bottom = [0, -1, -1];
  let right_back = [-1, 0, -1];

  let mut bottom_index = std::u32::MAX;
  let mut right_index = std::u32::MAX;
  let mut right_bottom_index = std::u32::MAX;
  let mut back_index = std::u32::MAX;
  let mut back_bottom_index = std::u32::MAX;
  let mut right_back_index = std::u32::MAX;

  // let mut current_pos =       [0.0, 0.0, 0.0];
  let mut bottom_pos = [0.0, 0.0, 0.0];
  let mut right_pos = [0.0, 0.0, 0.0];
  let mut right_bottom_pos = [0.0, 0.0, 0.0];
  let mut back_pos = [0.0, 0.0, 0.0];
  let mut back_bottom_pos = [0.0, 0.0, 0.0];
  let mut right_back_pos = [0.0, 0.0, 0.0];

  // TODO:
  //  Get the index of the position in the list of positions
  //  Potention solutions;
  //    Add to the positions
  //    Will determine the index of all: position, normals, indices, uvs
  //

  // Checking all 8 grid around the voxel
  let mut index = 0;
  for pos in possible_positions {
    if Utils::has_pos(&current_pos, pos, &bottom) {
      bottom_index = index;
      bottom_pos = pos.clone();
    }

    if Utils::has_pos(&current_pos, pos, &right) {
      right_index = index;
      right_pos = pos.clone();
    }
    if Utils::has_pos(&current_pos, pos, &right_bottom) {
      right_bottom_index = index;
      right_bottom_pos = pos.clone();
    }

    if Utils::has_pos(&current_pos, pos, &back) {
      back_index = index;
      back_pos = pos.clone();
    }
    if Utils::has_pos(&current_pos, pos, &back_bottom) {
      back_bottom_index = index;
      back_bottom_pos = pos.clone();
    }

    if Utils::has_pos(&current_pos, pos, &right_back) {
      right_back_index = index;
      right_back_pos = pos.clone();
    }
    index += 1;
  }

  let mut corner_000 = false; // Right bottom back
  let mut corner_001 = false; // Right bottom front
  let mut corner_100 = false; // Left bottom back
  let mut corner_010 = false; // Right top back
  if Utils::has_voxel(octree, &[x as u32, y as u32, z as u32], &[0, 0, 1]) {
    // 0, 0, 1 is right top edge
    corner_001 = true;
  }
  if Utils::has_voxel(octree, &[x as u32, y as u32, z as u32], &[0, 0, 0]) {
    // 0, 0, 0 is right bottom edge
    corner_000 = true;
  }
  if Utils::has_voxel(octree, &[x as u32, y as u32, z as u32], &[1, 0, 0]) {
    corner_100 = true;
  }
  if Utils::has_voxel(octree, &[x as u32, y as u32, z as u32], &[0, 1, 0]) {
    corner_010 = true;
  }

  let cur_index = get_index(&positions, &tmp_positions);
  if Utils::create_z_faces(right_index, right_bottom_index, bottom_index) && corner_000 ^ corner_001 {
    if corner_001 {
      let vertices = &[
        (current_pos, [0.0, 0.0, -1.0], [0.0, 1.0]),
        (bottom_pos, [0.0, 0.0, -1.0], [0.0, 1.0]),
        (right_pos, [0.0, 0.0, -1.0], [0.0, 1.0]),
        (right_bottom_pos, [0.0, 0.0, -1.0], [0.0, 1.0]),
      ];
      for (pos, nor, uv) in vertices.iter() {
        tmp_positions.push(*pos);
        normals.push(*nor);
        uvs.push(*uv);
      }

      indices.push(cur_index);
      indices.push(cur_index + 1);
      indices.push(cur_index + 2);

      indices.push(cur_index + 2);
      indices.push(cur_index + 1);
      indices.push(cur_index + 3);
    } else {
      let vertices = &[
        (current_pos, [0.0, 0.0, 1.0], [0.0, 1.0]),
        (bottom_pos, [0.0, 0.0, 1.0], [0.0, 1.0]),
        (right_pos, [0.0, 0.0, 1.0], [0.0, 1.0]),
        (right_bottom_pos, [0.0, 0.0, 1.0], [0.0, 1.0]),
      ];
      for (pos, nor, uv) in vertices.iter() {
        tmp_positions.push(*pos);
        normals.push(*nor);
        uvs.push(*uv);
      }
      indices.push(cur_index);
      indices.push(cur_index + 2);
      indices.push(cur_index + 1);

      indices.push(cur_index + 2);
      indices.push(cur_index + 3);
      indices.push(cur_index + 1);
    }
  }

  let cur_index = get_index(&positions, &tmp_positions);

  if Utils::create_x_faces(back_index, back_bottom_index, bottom_index) && corner_000 ^ corner_100 {
    if corner_000 {
      let vertices = &[
        (current_pos, [1.0, 0.0, 0.0], [0.0, 1.0]),
        (bottom_pos, [1.0, 0.0, 0.0], [0.0, 1.0]),
        (back_pos, [1.0, 0.0, 0.0], [0.0, 1.0]),
        (back_bottom_pos, [1.0, 0.0, 0.0], [0.0, 1.0]),
      ];
      for (pos, nor, uv) in vertices.iter() {
        tmp_positions.push(*pos);
        normals.push(*nor);
        uvs.push(*uv);
      }
      indices.push(cur_index);
      indices.push(cur_index + 1);
      indices.push(cur_index + 2);

      indices.push(cur_index + 2);
      indices.push(cur_index + 1);
      indices.push(cur_index + 3);
    } else {
      let vertices = &[
        (current_pos, [-1.0, 0.0, 0.0], [0.0, 1.0]),
        (bottom_pos, [-1.0, 0.0, 0.0], [0.0, 1.0]),
        (back_pos, [-1.0, 0.0, 0.0], [0.0, 1.0]),
        (back_bottom_pos, [-1.0, 0.0, 0.0], [0.0, 1.0]),
      ];
      for (pos, nor, uv) in vertices.iter() {
        tmp_positions.push(*pos);
        normals.push(*nor);
        uvs.push(*uv);
      }
      indices.push(cur_index);
      indices.push(cur_index + 2);
      indices.push(cur_index + 1);

      indices.push(cur_index + 2);
      indices.push(cur_index + 3);
      indices.push(cur_index + 1);
    }
  }

  let cur_index = get_index(&positions, &tmp_positions);

  if Utils::create_y_faces(right_back_index, right_index, back_index) && corner_000 ^ corner_010 {
    if corner_000 {
      let vertices = &[
        (current_pos, [0.0, 1.0, 0.0], [0.0, 1.0]),
        (back_pos, [0.0, 1.0, 0.0], [0.0, 1.0]),
        (right_pos, [0.0, 1.0, 0.0], [0.0, 1.0]),
        (right_back_pos, [0.0, 1.0, 0.0], [0.0, 1.0]),
      ];
      for (pos, nor, uv) in vertices.iter() {
        tmp_positions.push(*pos);
        normals.push(*nor);
        uvs.push(*uv);
      }
      indices.push(cur_index);
      indices.push(cur_index + 1);
      indices.push(cur_index + 2);

      indices.push(cur_index + 2);
      indices.push(cur_index + 1);
      indices.push(cur_index + 3);
    } else {
      let vertices = &[
        (current_pos, [0.0, -1.0, 0.0], [0.0, 1.0]),
        (back_pos, [0.0, -1.0, 0.0], [0.0, 1.0]),
        (right_pos, [0.0, -1.0, 0.0], [0.0, 1.0]),
        (right_back_pos, [0.0, -1.0, 0.0], [0.0, 1.0]),
      ];
      for (pos, nor, uv) in vertices.iter() {
        tmp_positions.push(*pos);
        normals.push(*nor);
        uvs.push(*uv);
      }
      indices.push(cur_index);
      indices.push(cur_index + 2);
      indices.push(cur_index + 1);

      indices.push(cur_index + 2);
      indices.push(cur_index + 3);
      indices.push(cur_index + 1);
    }
  }
  (tmp_positions, normals, indices, uvs)
}
