use voxels::{data::{voxel_octree::{VoxelOctree, VoxelMode, ParentValueType}, surface_nets::{VoxelReuse, GridPosition}}, utils::get_length};
use criterion::{black_box, criterion_group, criterion_main, Criterion};

pub fn bench_get_surface_nets(c: &mut Criterion) {
  let depth = 4;
  let default_value = 100;
  let size = (2 as u32).pow(depth as u32);
  let mut new_value = 0;
  let mut data = Vec::new();
  for x in 0..size {
    for y in 0..size {
      for z in 0..size {
        new_value = if new_value == 255 { 0 } else { new_value + 1 };
        data.push([x, y, z, new_value]);
      }
    }
  }

  let octree = VoxelOctree::new_from_3d_array(
    default_value, 
    depth, 
    &data, 
    ParentValueType::Lod
  );

  let mut voxel_reuse = VoxelReuse::new(depth as u32, 3);

  c.bench_function("get_surface_nets", |b| {
    b.iter(|| {
      octree.compute_mesh2(VoxelMode::SurfaceNets, &mut voxel_reuse);
    })
  });
}

pub fn bench_octree_get_voxel(c: &mut Criterion) {
  c.bench_function("octree_get_voxel", |b| {
    let depth = 4;
    let default_value = 100;
    let tmp_octree = VoxelOctree::new(default_value, depth);
    let size = tmp_octree.get_size();
    let mut new_value = 0;

    let mut data = Vec::new();
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };
          data.push([x, y, z, new_value]);
        }
      }
    }
    let octree = VoxelOctree::new_from_3d_array(default_value, depth, &data, ParentValueType::Lod);

    b.iter(|| {
      for x in 0..size {
        for y in 0..size {
          for z in 0..size {
            octree.get_voxel(x, y, z);
          }
        }
      }
    })
  });
}

criterion_group!(
  benches,
  bench_get_surface_nets,
  bench_octree_get_voxel
);
criterion_main!(benches);
