pub mod cubic;
pub mod dual_contour;
pub mod surface_nets;
pub mod voxel_octree;


pub const CUBE_EDGES: [(usize, usize); 12] = [
  (0b000, 0b001),
  (0b000, 0b010),
  (0b000, 0b100),
  (0b001, 0b011),
  (0b001, 0b101),
  (0b010, 0b011),
  (0b010, 0b110),
  (0b011, 0b111),
  (0b100, 0b101),
  (0b100, 0b110),
  (0b101, 0b111),
  (0b110, 0b111),
];