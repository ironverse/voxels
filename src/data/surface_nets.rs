use parry3d::{na::coordinates::X, math::Point};
use crate::utils::{coord_to_index, Utils};
use super::voxel_octree::*;
use crate::data::CUBE_EDGES;

const CURRENT: [i8; 3] = [0, 0, 0];
const LEFT: [i8; 3] = [1, 0, 0];
const RIGHT: [i8; 3] = [-1, 0, 0];
const UP: [i8; 3] = [0, 1, 0];
const DOWN: [i8; 3] = [0, -1, 0];
const FRONT: [i8; 3] = [0, 0, 1];

const BACK: [i8; 3] = [0, 0, -1];
const BACK_DOWN: [i8; 3] = [0, -1, -1];

const RIGHT_DOWN: [i8; 3] = [-1,-1, 0];
const RIGHT_BACK: [i8; 3] = [-1, 0,-1];


#[derive(Clone)]
struct GridPosition {
  index: u32,
  pos: Option<[f32; 3]>,
}

pub fn get_surface_nets(octree: &VoxelOctree, start_pos: &[f32; 3]) -> MeshData {
  let mut possible_positions = Vec::new();
  let mut positions = Vec::new();
  let mut normals = Vec::new();
  let mut uvs = Vec::new();
  let mut indices = Vec::new();

  // Checking for each grid
  let start = 1;
  let end = octree.get_size() - 2;
  for x in start..end {
    for y in start..end {
      for z in start..end {
        // Process the defined voxels first
        // Then do the identification for the voxels via octree later

        // println!("pos {}, {}, {}", x, y, z);
        let (pos, nor) = get_average_vertex_pos(octree, x, y, z);
        if pos[0] == 0.0 {
          continue;
        }
        // println!("after pos {}, {}, {}", x, y, z);
        possible_positions.push(pos);

        let mut indices1 = get_indices(
          octree,
          x as i32,
          y as i32,
          z as i32,
          pos,
          &possible_positions,
        );
        positions.push([
          pos[0] + start_pos[0],
          pos[1] + start_pos[1],
          pos[2] + start_pos[2],
        ]);
        normals.push(nor);
        uvs.push([0.0, 0.0]);
        indices.append(&mut indices1);
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

fn get_average_vertex_pos(octree: &VoxelOctree, x: u32, y: u32, z: u32) -> ([f32; 3], [f32; 3]) {
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

pub fn estimate_surface_edge_intersection(
  offset1: usize,
  offset2: usize,
  value1: f32,
  value2: f32,
) -> Option<[f32; 3]> {
  if (value1 < 0.0) == (value2 < 0.0) {
    return None;
  }

  let interp1 = value1 / (value1 - value2);
  let interp2 = 1.0 - interp1;
  let position = [
    (offset1 & 1) as f32 * interp2 + (offset2 & 1) as f32 * interp1,
    ((offset1 >> 1) & 1) as f32 * interp2 + ((offset2 >> 1) & 1) as f32 * interp1,
    ((offset1 >> 2) & 1) as f32 * interp2 + ((offset2 >> 2) & 1) as f32 * interp1,
  ];

  Some(position)
}

fn get_indices(
  octree: &VoxelOctree,
  x: i32,
  y: i32,
  z: i32,
  current_pos: [f32; 3],
  possible_positions: &Vec<[f32; 3]>,
) -> Vec<u32> {
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

  //  Right hand rule voxel detection
  let mut corner_000 = false; // Right bottom back
  let mut corner_001 = false; // Right bottom front
  let mut corner_100 = false; // Left bottom back
  let mut corner_010 = false; // Right top back

  let cur_pos_u32 = [x as u32, y as u32, z as u32];
  if has_voxel(octree, &cur_pos_u32, &[0, 0, 1]) {
    corner_001 = true;
  }
  if has_voxel(octree, &cur_pos_u32, &[0, 0, 0]) {
    corner_000 = true;
  }
  if has_voxel(octree, &cur_pos_u32, &[1, 0, 0]) {
    corner_100 = true;
  }
  if has_voxel(octree, &cur_pos_u32, &[0, 1, 0]) {
    corner_010 = true;
  }


  /*
    Inverted right hand rule detection creation of mesh
  */
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

  //  There should be a position in the grid
  //  Either right bottom back or front
  if Utils::create_z_faces(right_index, right_bottom_index, bottom_index) && corner_000 ^ corner_001 {
    //  Normal towards positive z
    //  Else: negative z
    if corner_001 {
      indices.push(current_index);
      indices.push(bottom_index);
      indices.push(right_index);

      indices.push(right_index);
      indices.push(bottom_index);
      indices.push(right_bottom_index);
    } else {
      indices.push(current_index);
      indices.push(right_index);
      indices.push(bottom_index);

      indices.push(right_index);
      indices.push(right_bottom_index);
      indices.push(bottom_index);
    }
  }

  if Utils::create_x_faces(back_index, back_bottom_index, bottom_index) && corner_000 ^ corner_100 {
    if corner_000 {
      indices.push(current_index);
      indices.push(bottom_index);
      indices.push(back_index);

      indices.push(back_index);
      indices.push(bottom_index);
      indices.push(back_bottom_index);
    } else {
      indices.push(current_index);
      indices.push(back_index);
      indices.push(bottom_index);

      indices.push(back_index);
      indices.push(back_bottom_index);
      indices.push(bottom_index);
    }
  }

  if Utils::create_y_faces(right_back_index, right_index, back_index) && corner_000 ^ corner_010 {
    if corner_000 {
      indices.push(current_index);
      indices.push(back_index);
      indices.push(right_index);

      indices.push(right_index);
      indices.push(back_index);
      indices.push(right_back_index);
    } else {
      indices.push(current_index);
      indices.push(right_index);
      indices.push(back_index);

      indices.push(right_index);
      indices.push(right_back_index);
      indices.push(back_index);
    }
  }

  indices
}




/*

*/
pub fn get_surface_nets2(octree: &VoxelOctree) -> MeshData {
  let mut positions = Vec::new();
  let mut normals = Vec::new();
  let mut uvs = Vec::new();
  let mut indices = Vec::new();
  let mut grid_pos = Vec::new();


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


  // Checking for each grid
  let start = 0;
  let end = octree.get_size() - 1;
  for x in start..end {
    for y in start..end {
      for z in start..end {
        // Process the defined voxels first
        // Then do the identification for the voxels via octree later

        // println!("pos {}, {}, {}", x, y, z);
        let (pos_op, nor) = get_average_vertex_pos2(&voxels, voxel_start, voxel_end, x, y, z);
        grid_pos.push(GridPosition {
          index: u32::MAX,
          pos: pos_op.clone()
        });

        let index = grid_pos.len() - 1;
        let grid = &mut grid_pos[index];
        
        if pos_op.is_none() {
          continue;
        }

        let pos = pos_op.unwrap();
        positions.push(pos);
        grid.index = (positions.len() - 1) as u32;
        
        set_indices2(
          octree,
          x,
          y,
          z,
          start,
          end,
          &voxels,
          voxel_start,
          voxel_end,
          &grid_pos,
          &mut indices
        );
        normals.push(nor);
        uvs.push([0.0, 0.0]);
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

/** 
 * Should only access 0-14 indices to determine if there is vertex/grid position
 * Checking is using right hand rule
*/
fn get_average_vertex_pos2(
  voxels: &Vec<u8>,
  start: u32,
  end: u32,
  x: u32,
  y: u32,
  z: u32,
) -> (Option<[f32; 3]>, [f32; 3]) {
  let mut avg_pos = None;
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
    let x = sum[0] / count as f32 + x as f32 + 0.0;
    let y = sum[1] / count as f32 + y as f32 + 0.0;
    let z = sum[2] / count as f32 + z as f32 + 0.0;
    // println!("avg_pos {:?}", avg_pos);
    avg_pos = Some([x, y, z]);
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



/*
  TODO:
    Fix still creating unnecessary mesh at the end of index(15)
      If the value is just in the end of the index:
        If x = 15
          Disable mesh facing negative direction
        If x = 0
          Disable mesh facing positive direction
        Do the same with other axes
      Conditions
        If on the edge of index
*/
/*
  Draft:
  Conditions to create indices for mesh
    Based on the grid voxel value?

  
  Conditions
    Where to face the mesh by defining the indices
    Current:
      For now, use the grid voxel value as indicator:
        Outward direction from the value
        Solve problem as we go along

      v1
        Indicator is using grid voxel value: current + axis_alignment
          TOD0: Prove to the other axis
          If z
            current + 001
            Ex:
              Grid voxel: [1, 1, 1]

              Grid coord: [1, 1, 0]
              Face: Negative z
              ---
              Grid coord: [1, 1, 1]
              Face: Positive z
*/
fn set_indices2(
  octree: &VoxelOctree,
  x: u32, // Grid coord of VoxelOctree
  y: u32, // Grid coord of VoxelOctree
  z: u32, // Grid coord of VoxelOctree
  start: u32,
  end: u32,
  voxels: &Vec<u8>,
  voxel_start: u32,
  voxel_end: u32,
  grid_pos: &Vec<GridPosition>,
  indices: &mut Vec<u32>
) {
  let grid_coord = &[x, y, z];
  set_indices_x(
    indices,
    octree,
    grid_pos,
    grid_coord,
    start,
    end,
    voxels,
    voxel_start,
    voxel_end
  );

  set_indices_y(
    indices,
    octree,
    grid_pos,
    grid_coord,
    start,
    end,
    voxels,
    voxel_start,
    voxel_end
  );

  set_indices_z(
    indices,
    octree,
    grid_pos,
    grid_coord,
    start,
    end,
    voxels,
    voxel_start,
    voxel_end
  );
}


/* 
  TODO:
    Fix still creating unnecessary mesh at the end of index(15)
      If the value is just in the end of the index:
        If x = 15
          Disable mesh facing negative direction
        If x = 0
          Disable mesh facing positive direction
        Do the same with other axes
      Conditions
        If on the edge of index
*/
fn set_indices_x(
  indices: &mut Vec<u32>,
  octree: &VoxelOctree,
  grids: &Vec<GridPosition>,
  grid_coord: &[u32; 3],
  start: u32,
  end: u32,
  voxels: &Vec<u8>,
  voxel_start: u32,
  voxel_end: u32,
) {
  /*
    Inverted right hand rule detection creation of mesh
  */
  let current = coord_to_index2(grids, &grid_coord, &CURRENT, start, end);
  let back = coord_to_index2(grids, &grid_coord, &BACK, start, end);
  let back_down = coord_to_index2(grids, &grid_coord, &BACK_DOWN, start, end);
  let down = coord_to_index2(grids, &grid_coord, &DOWN, start, end);

  // if create_x_faces(back_index, back_bottom_index, bottom_index) && corner_000 ^ corner_100 {
  if has_position_indices_for_x(back, back_down, down) {
    let x = grid_coord[0];
    let y = grid_coord[1];
    let z = grid_coord[2];

    // let face_left = octree.get_voxel(x, y, z) == 1; // Current
    // let face_right = octree.get_voxel(x + 1, y, z) == 1; // Left

    let index = coord_to_index(x, y, z, voxel_start, voxel_end); // Current
    let face_left = voxels[index] == 1;

    let index = coord_to_index(x + 1, y, z, voxel_start, voxel_end); // Left
    let face_right = voxels[index] == 1;

    let create = face_left ^ face_right;  // Only one should be true
    if create {
      let start = 0;
      if face_left && x != start {
        indices.push(current);
        indices.push(down);
        indices.push(back);
  
        indices.push(back);
        indices.push(down);
        indices.push(back_down);
      } 
      
      // println!("grid_coord {:?}", grid_coord);
      let end_index = octree.get_size() - 2;
      if face_right && x != end_index {
        indices.push(current);
        indices.push(back);
        indices.push(down);
  
        indices.push(back);
        indices.push(back_down);
        indices.push(down);
      }
    }
  }
}

fn set_indices_y(
  indices: &mut Vec<u32>,
  octree: &VoxelOctree,
  grids: &Vec<GridPosition>,
  grid_coord: &[u32; 3],
  start: u32,
  end: u32,
  voxels: &Vec<u8>,
  voxel_start: u32,
  voxel_end: u32,
) {

  let current = coord_to_index2(grids, &grid_coord, &CURRENT, start, end);
  let right_back = coord_to_index2(grids, &grid_coord, &RIGHT_BACK, start, end);
  let right = coord_to_index2(grids, &grid_coord, &RIGHT, start, end);
  let back = coord_to_index2(grids, &grid_coord, &BACK, start, end);

  if has_position_indices_for_y(right_back, right, back) {
    let x = grid_coord[0];
    let y = grid_coord[1];
    let z = grid_coord[2];

    // let face_up = octree.get_voxel(x, y, z) == 1;
    // let face_down = octree.get_voxel(x, y + 1, z) == 1; 

    let index = coord_to_index(x, y, z, voxel_start, voxel_end); // Grid voxel below: Note: Should be current?
    let face_up = voxels[index] == 1;

    let index = coord_to_index(x, y + 1, z, voxel_start, voxel_end); // Grid voxel on top
    let face_down = voxels[index] == 1;

    let create = face_up ^ face_down;
    if create {
      let start = 0;
      if face_up && y != start {
        indices.push(current);
        indices.push(back);
        indices.push(right);
  
        indices.push(right);
        indices.push(back);
        indices.push(right_back);
      } 
      
      let end = octree.get_size() - 2;
      if face_down && y != end {
        indices.push(current);
        indices.push(right);
        indices.push(back);
  
        indices.push(right);
        indices.push(right_back);
        indices.push(back);
      }
    }

  }
}

fn set_indices_z(
  indices: &mut Vec<u32>,
  octree: &VoxelOctree,
  grids: &Vec<GridPosition>,
  grid_coord: &[u32; 3],
  start: u32,
  end: u32,
  voxels: &Vec<u8>,
  voxel_start: u32,
  voxel_end: u32,
) {

  let current = coord_to_index2(grids, &grid_coord, &CURRENT, start, end);
  let right = coord_to_index2(grids, &grid_coord, &RIGHT, start, end);
  let right_down = coord_to_index2(grids, &grid_coord, &RIGHT_DOWN, start, end);
  let down = coord_to_index2(grids, &grid_coord, &DOWN, start, end);

  if has_position_indices_for_z(right, right_down, down) {
    let x = grid_coord[0];
    let y = grid_coord[1];
    let z = grid_coord[2];

    // let face_front = octree.get_voxel(x, y, z) == 1; // Current
    // let face_back = octree.get_voxel(x, y, z + 1) == 1; // Forward

    let index = coord_to_index(x, y, z, voxel_start, voxel_end); // Current
    let face_front = voxels[index] == 1;

    let index = coord_to_index(x, y, z + 1, voxel_start, voxel_end); // Forward
    let face_back = voxels[index] == 1;
    
    // For now, never allow if both have values, defer solving it later
    let create = face_front ^ face_back;
    if create {
      let start = 0;
      if face_front && z != start { // Face forward
        // Clockwise: Facing z positive
        indices.push(current);
        indices.push(right);
        indices.push(down);

        indices.push(right);
        indices.push(right_down);
        indices.push(down);
      }

      let end_index = octree.get_size() - 2;
      if face_back && z != end_index { // Face backward
        // Counter-clockwise
        indices.push(current);
        indices.push(down);
        indices.push(right);

        indices.push(right);
        indices.push(down);
        indices.push(right_down);
      }
    }
  }
}





fn coord_to_index2(
  grid_pos: &Vec<GridPosition>,
  cur: &[u32; 3], 
  dir: &[i8; 3], 
  start: u32, 
  end: u32
) -> u32 {
  let coord_op = grid_coord(cur, dir);
  if coord_op.is_none() {
    return u32::MAX;
  }
  let coord = coord_op.unwrap();
  let mut index = coord_to_index(
    coord[0] as u32, 
    coord[1] as u32, 
    coord[2] as u32, 
    start, 
    end
  );

  let grid = &grid_pos[index];
  grid.index
}

fn grid_coord(cur: &[u32; 3], dir: &[i8; 3]) -> Option<[i32; 3]> {
  let x = cur[0] as i32 + dir[0] as i32;
  let y = cur[1] as i32 + dir[1] as i32;
  let z = cur[2] as i32 + dir[2] as i32;
  if x < 0 || y < 0 || z < 0 {
    return None;
  }
  Some([x, y, z])
}

pub fn has_position_indices_for_x(back_index: u32, back_bottom_index: u32, bottom_index: u32) -> bool {
  back_index != std::u32::MAX && 
  back_bottom_index != std::u32::MAX && 
  bottom_index != std::u32::MAX
}

pub fn has_position_indices_for_y(right_back_index: u32, right_index: u32, back_index: u32) -> bool {
  right_back_index != std::u32::MAX && right_index != std::u32::MAX && back_index != std::u32::MAX
}


pub fn has_position_indices_for_z(right_index: u32, right_bottom_index: u32, bottom_index: u32) -> bool {
  right_index != std::u32::MAX
    && right_bottom_index != std::u32::MAX
    && bottom_index != std::u32::MAX
}

fn has_voxel(octree: &VoxelOctree, pos: &[u32; 3], dir: &[u32; 3]) -> bool {
  octree.get_voxel(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]) > 0
}


fn coord_to_index2_test(
  grid_pos: &Vec<GridPosition>,
  cur: &[u32; 3], 
  dir: &[i8; 3], 
  start: u32, 
  end: u32
) -> GridPosition {
  let coord_op = grid_coord(cur, dir);
  // if coord_op.is_none() {
  //   return u32::MAX;
  // }
  let coord = coord_op.unwrap();
  let mut index = coord_to_index(
    coord[0] as u32, 
    coord[1] as u32, 
    coord[2] as u32, 
    start, 
    end
  );

  let grid = &grid_pos[index];
  grid.clone()
}



/*
  Code for client side
*/
#[derive(Clone)]
pub struct MeshColliderData {
  pub positions: Vec<Point<f32>>,
  pub indices: Vec<[u32; 3]>,
}

/* TODO: Defer optimizing code */
/*
pub fn client_create_mesh(f: fn(&[f32; 3]), octree: &VoxelOctree) -> MeshColliderData {
  let mut positions = Vec::new();
  let mut normals = Vec::new();
  let mut uvs = Vec::new();
  let mut indices = Vec::new();

  let mut grid_pos = Vec::new();

  // Checking for each grid
  let start = 0;
  let end = octree.get_size() - 1;
  for x in start..end {
    for y in start..end {
      for z in start..end {
        // Process the defined voxels first
        // Then do the identification for the voxels via octree later

        // println!("pos {}, {}, {}", x, y, z);
        let (pos_op, nor) = get_average_vertex_pos2(octree, x, y, z);
        
        grid_pos.push(GridPosition {
          index: u32::MAX,
          pos: pos_op.clone()
        });

        let index = grid_pos.len() - 1;
        let grid = &mut grid_pos[index];
        
        if pos_op.is_none() {
          continue;
        }
        
        let pos = pos_op.unwrap();
        positions.push(Point::new(pos[0], pos[1], pos[2]));
        grid.index = (positions.len() - 1) as u32;
        
        set_indices2(
          octree,
          x,
          y,
          z,
          start,
          end,
          &grid_pos,
          &mut indices
        );
        normals.push(nor);
        uvs.push([0.0, 0.0]);

        f(&pos);
      }
    }
  }
  MeshColliderData {
    positions: positions,
    indices: indices,
  }
}
*/


pub fn create_collider_mesh(octree: &VoxelOctree) -> MeshColliderData {
  let mesh = get_surface_nets2(octree);

  let mut positions = Vec::new();
  let mut indices = Vec::new();
  
  for pos in mesh.positions.iter() {
    positions.push(Point::new(pos[0], pos[1], pos[2]));
  }
  
  for ind in mesh.indices.chunks(3) {
    // println!("i {:?}", ind);
    indices.push([ind[0], ind[1], ind[2]]);
  }


  MeshColliderData {
    positions: positions,
    indices: indices,
  }
}