use voxels::data::voxel_octree::{VoxelOctree, VoxelMode, ParentValueType};
use criterion::{black_box, criterion_group, criterion_main, Criterion};

pub fn bench_get_surface_nets(c: &mut Criterion) {
  let depth = 4;
  let default_value = 100;
  let size = (2 as u32).pow(depth.into());
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

  c.bench_function("get_surface_nets", |b| {
    b.iter(|| {
      octree.compute_mesh2(VoxelMode::SurfaceNets);
    })
  });
}

criterion_group!(
  benches,
  bench_get_surface_nets,
);
criterion_main!(benches);
