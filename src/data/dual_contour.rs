use super::voxel_octree::MeshData;
use super::voxel_octree::*;

pub fn get_dual_contour(octree: &VoxelOctree, _start_pos: &[f32; 3]) -> MeshData {
  let positions = Vec::new();
  let normals = Vec::new();
  let uvs = Vec::new();
  let indices = Vec::new();

  // Checking for each grid
  for _x in 0..octree.get_size() {
    for _y in 0..octree.get_size() {
      for _z in 0..octree.get_size() {

        // let (dists, voxel_count) = get_intersection(x, y, z);

        // let pos = get_pos(octree, x, y, z);
        // if pos[0] > -std::f32::EPSILON && pos[0] < std::f32::EPSILON {
        //   continue;
        // }
        // possible_positions.push(pos);

        // let (mut positions1, mut normals1, mut uvs1, mut indices1) = get_indices(
        //   octree,
        //   x as i32,
        //   y as i32,
        //   z as i32,
        //   pos,
        //   &possible_positions,
        //   &positions,
        // );
        // positions.append(&mut positions1);
        // indices.append(&mut indices1);
        // normals.append(&mut normals1);
        // uvs.append(&mut uvs1);
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

// fn get_intersections(x: u32, y: u32, z: u32) -> ([f32; 8], u32) {
//   let mut voxel_count = 0;
//   let mut dists = [1.0; 8];
//   // Checking all corners for voxel values
//   for x_offset in 0..2 {
//     for y_offset in 0..2 {
//       for z_offset in 0..2 {
//         let corner_x = x_offset + x;
//         let corner_y = y_offset + y;
//         let corner_z = z_offset + z;
//         let voxel = octree.get_voxel(corner_x, corner_y, corner_z);
//         if voxel > 0 {
//           voxel_count += 1;
//           let x_index = x_offset;
//           let y_index = y_offset << 1;
//           let z_index = z_offset << 2;
//           let corner_index = x_index as usize + y_index as usize + z_index as usize;
//           // println!("x y z {} {} {}", x_index, y_index, z_index);
//           dists[corner_index] = -1.0;
//         }
//       }
//     }
//   }
//   (dist, voxel_count)
// }

// fn get_changes() {
//   for (offset1, offset2) in CUBE_EDGES.iter() {
//     if let Some(intersection) = estimate_surface_edge_intersection(
//       *offset1,
//       *offset2,
//       dists[*offset1],
//       dists[*offset2],
//     ) {
//       println!("intersection {:?}", intersection);
//     }
//   }
// }
