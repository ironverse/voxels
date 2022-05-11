use hashbrown::HashMap;

use crate::utils::coord_to_index;
use crate::utils::get_length;

use super::cubic::*;
use super::dual_contour::*;
use super::surface_nets::*;

#[derive(PartialEq, Clone, Copy)]
pub enum ParentValueType {
  Lod,
  FillEmptyChildrenWithDefaultValue,
  DefaultValue
}

#[derive(Clone, Copy, Debug)]
pub enum VoxelMode {
  Cube,
  SurfaceNets,
  DualContour,
}

#[derive(Clone, Debug, Default)]
pub struct VoxelOctree {
  pub data: Vec<u8>,
  pub size: u32,
  pub layers: Vec<usize>,
  pub layer_mappings: Vec<Vec<usize>>,
  pub layer_section_cache: Vec<(usize, usize)>,
}

#[derive(Default, Clone, Debug)]
pub struct MeshData {
  pub positions: Vec<[f32; 3]>,
  pub normals: Vec<[f32; 3]>,
  pub uvs: Vec<[f32; 2]>,
  pub indices: Vec<u32>,
}

#[derive(Default, Clone, Debug)]
struct Node {
  pub children: [usize; 8],
  pub descriptor: u8,
  pub default_value: u8,
  pub values: Vec<u8>,
}

#[derive(Default, Clone, Debug)]
struct LayerDetail {
  pub default_values: Vec<u8>,
  pub descriptors: Vec<u8>,
  pub mapping: Vec<usize>,
}

/*
  TODO:
    Make layer_mappings 1 dimensional array(Nope, complicated, defer later)
      Refactor get_voxel() (defer)
      Refactor calculate_layer_mappings() (defer)
      Fix set_voxel()
      Update unit tests
*/

impl VoxelOctree {
  //returns a new byte array representation of an octree. Size is 2^depth. min depth is 1. 2^2=4, 2^4=16, 2^6=64, 2^8=256,
  pub fn new(default_value: u8, depth: u8) -> Self {
    VoxelOctree::new_from_bytes(vec![depth, default_value, 0b_0000_0000_u8])
  }
  pub fn new_from_bytes(data: Vec<u8>) -> Self {
    let size = (2 as u32).pow(data[0].into());
    let mut new = Self {
      data,
      size,
      ..Default::default()
    };
    new.calculate_layer_mappings();
    new
  }

  pub fn new_from_3d_array(
    default_value: u8, depth: u8, voxels: &Vec<[u32; 4]>, mode: ParentValueType
  ) -> Self {
    let mut octree = VoxelOctree::new(default_value, depth);
    let len = get_length(depth) + 1;
    let mut nodes: Vec<Option<Node>> = Vec::new();
    for _ in 0..len {
      nodes.push(None);
    }

    let root_index = get_num_key(&vec![]);
    nodes[root_index].insert(Node::default());

    let mut key = Vec::new();
    let start_size = octree.size / 2;
    let mut parent_index = root_index;
    for v in voxels.iter() {
      let mut x = v[0];
      let mut y = v[1];
      let mut z = v[2];

      let mut size = start_size;

      parent_index.clone_from(&root_index);
      key.clear();
      while size > 0 {
        let mut bit: u8 = 0;

        if x >= size {
          bit += 1;
          x -= size;
        }
        if y >= size {
          bit += 2;
          y -= size;
        }
        if z >= size {
          bit += 4;
          z -= size;
        }

        size /= 2;

        let mut index = 0;
        if size == 0 {
          index = get_num_key(&key);
          process_leaf(&index, &bit, v[3] as u8, &mut nodes);
        } else {
          key.push(bit);
          index = get_num_key(&key);
          process_branch(&index, &parent_index, &bit, &mut nodes);
        }
        parent_index.clone_from(&index);
      }
    }

    let mut layer_details = Vec::new();
    for _ in 0..depth {
      layer_details.push(LayerDetail::default());
    }
    let mut values = Vec::new();
    let root = nodes.get(root_index).unwrap().as_ref().unwrap();

    if octree.get_depth() == 1 {
      values.clone_from(&root.values);
    } else {
      calc_child(
        &mut nodes,
        &root_index,
        0,
        default_value as u8,
        &mut layer_details,
        &mut values,
        mode
      );
    }

    octree.data = vec![octree.data[0]];
    octree.layer_mappings.clear();
    for layer_detail in layer_details.iter() {
      octree.data.extend(layer_detail.default_values.iter());
      octree.data.extend(layer_detail.descriptors.iter());
      octree.layer_mappings.push(layer_detail.mapping.clone());
    }

    octree.data.extend(values.iter());

    octree.calculate_start_layer_indices();
    octree.cache_layer_sections();
    octree
  }

  pub fn set_voxel(&mut self, mut x: u32, mut y: u32, mut z: u32, new_value: u8) {
    let mut size = self.size / 2;
    let mut local_layer_index = 0;
    let mut prev_layer_index = 0;
    let mut local_layer_mapped_index = 0;
    let mut next_layer_index = 0;

    let default_value = self.data[1];

    for layer in 0..(self.get_depth() + 1) as usize {
      let (layer_start, layer_size) = self.get_layer_section(layer);
      let desc_start = layer_start + layer_size / 2;
      let descriptor_index = desc_start + local_layer_index;

      // Coordinate already exists, overwrite the voxel value
      if layer == self.get_depth() as usize {
        let desc_index = desc_start + local_layer_index;
        self.data[desc_index] = new_value;
        return;
      }

      let descriptor = self.data[descriptor_index];
      if layer < self.get_depth().into() {
        local_layer_mapped_index = self.layer_mappings[layer][local_layer_index];
        next_layer_index += local_layer_mapped_index;
      }

      prev_layer_index = local_layer_index;
      local_layer_index = next_layer_index;
      next_layer_index = 0;

      let mut bit: u8 = 0;
      if x >= size {
        bit += 1;
        x -= size;
      }
      if y >= size {
        bit += 2;
        y -= size;
      }
      if z >= size {
        bit += 4;
        z -= size;
      }
      size /= 2;

      let branch = 1 << bit;
      let has_descriptor = descriptor & branch == branch;
      if has_descriptor {
        local_layer_index += branch_index_reverse(descriptor, branch);
        continue;
      }

      let is_branch = layer as u8 != self.get_depth() - 1;
      self.data[descriptor_index] = descriptor | branch;
      let new_descriptor = self.data[descriptor_index];
      local_layer_index += branch_index_reverse(new_descriptor, branch);

      if is_branch {
        self.calculate_layer_mappings();
        let (next_start, next_size) = self.get_layer_section(layer + 1);
        let next_desc_start = next_start + next_size / 2;

        let branch_index = branch_index_reverse(new_descriptor, branch);
        let local_index = self.layer_mappings[layer][prev_layer_index] + branch_index;

        let default_index = next_start + local_index;
        let desc_index = next_desc_start + local_index;

        self.data.insert(default_index, default_value);
        self.data.insert(desc_index, 0);

        local_layer_index = local_index;
        continue;
      }

      self.calculate_layer_mappings();
      let branch_index = branch_index_reverse(new_descriptor, branch);
      let local_index = self.layer_mappings[layer][prev_layer_index] + branch_index;

      let (next_start, next_size) = self.get_layer_section(layer + 1);
      let default_index = next_start + local_index;
      self.data.insert(default_index, new_value);
      local_layer_index = local_index;
    }

    panic!("error setting voxel");
  }

  pub fn get_voxel(&self, mut x: u32, mut y: u32, mut z: u32) -> u8 {
    check_out_of_bound_access(self.size, x, y, z);

    let mut size = self.size / 2;
    let mut local_layer_index = 0;
    let mut prev_layer_index = 0;
    let mut local_layer_mapped_index = 0;
    let mut next_layer_index = 0;

    let depth = self.get_depth() as usize;
    for layer in 0..depth + 1 as usize {
      let (layer_start, layer_size) = self.get_layer_section(layer);
      let desc_start = layer_start + layer_size / 2;
      let descriptor_index = desc_start + local_layer_index;

      let reached_slice_by_lod = descriptor_index >= self.data.len();
      if reached_slice_by_lod {
        let default_value_index = layer_start + local_layer_index;
        return self.data[default_value_index];
      }

      let descriptor = self.data[descriptor_index];
      if layer == depth {
        return descriptor;
      }

      if layer < depth {
        local_layer_mapped_index = self.layer_mappings[layer][local_layer_index];
        next_layer_index += local_layer_mapped_index;
      }

      prev_layer_index = local_layer_index;
      local_layer_index = next_layer_index;
      next_layer_index = 0;

      let mut bit: u8 = 0;
      if x >= size {
        bit += 1;
        x -= size;
      }
      if y >= size {
        bit += 2;
        y -= size;
      }
      if z >= size {
        bit += 4;
        z -= size;
      }
      size /= 2;

      let branch = 1 << bit;
      let has_descriptor = descriptor & branch == branch;
      if has_descriptor {
        local_layer_index += branch_index_reverse(descriptor, branch);
        continue;
      }

      let default_value_index = layer_start + prev_layer_index;
      return self.data[default_value_index];
    }
    panic!("error getting voxel");
  }

  fn calculate_start_layer_indices(&mut self) {
    let mut start_layer_index = 1;
    let mut total_branches = 1;

    let mut layer_start_indices = vec![start_layer_index];
    for _ in 0..(self.get_depth()) as usize {
      let start = start_layer_index;

      if total_branches == 0 {
        break;
      }

      start_layer_index += (total_branches * 2);
      layer_start_indices.push(start_layer_index);

      let mut next_layer_total_branches = 0;
      for index in 0..total_branches {
        let desc_index = start + (index + total_branches);
        if desc_index >= self.data.len() {
          break;
        }

        let desc = self.data[desc_index];
        next_layer_total_branches += VoxelOctree::get_branch_count(desc);
      }

      total_branches = next_layer_total_branches;
    }
    self.layers = layer_start_indices;
  }

  pub fn calculate_layer_mappings(&mut self) {
    self.calculate_start_layer_indices();
    self.layer_section_cache.clear();

    self.layer_mappings.clear();
    for layer in 0..(self.get_depth() + 1) as usize {
      let (layer_start, layer_size) = self.get_layer_section(layer);
      let total_desc = layer_size / 2;
      let desc_start = layer_start + total_desc;

      self.layer_mappings.push(Vec::new());
      self.layer_mappings[layer].push(0);
      if total_desc == 0 {
        break;
      }

      for local_index in 0..total_desc {
        let index = desc_start + local_index;
        if index >= self.data.len() {
          break;
        }

        let prev_branches = self.layer_mappings[layer][self.layer_mappings[layer].len() - 1];
        let branches = VoxelOctree::get_branch_count(self.data[index]);
        self.layer_mappings[layer].push(prev_branches + branches);
      }
    }
  }

  fn get_branch_count(descriptor: u8) -> usize {
    let mut branch_count = 0;
    for bit in 0..8 {
      let branch = 1 << bit;
      if descriptor & branch == branch {
        branch_count += 1;
      }
    }
    branch_count
  }

  pub fn get_depth(&self) -> u8 {
    self.data[0]
  }

  pub fn get_size(&self) -> u32 {
    self.size
  }

  //Eventually, client will use this logic in a compute shader instead. (But region will still need this for physics meshes)
  pub fn compute_mesh(&self, mode: VoxelMode, start_pos: &[f32; 3]) -> MeshData {
    match mode {
      VoxelMode::SurfaceNets => get_surface_nets(self, start_pos),
      VoxelMode::Cube => get_cube(self, start_pos),
      VoxelMode::DualContour => get_dual_contour(self, start_pos),
    }
  }

  pub fn compute_mesh2(&self, mode: VoxelMode) -> MeshData {
    match mode {
      VoxelMode::SurfaceNets => get_surface_nets2(self),
      _ => panic!("VoxelMode {:?} implementation not existing yet", mode),
    }
  }

  pub fn is_empty(&self) -> bool {
    self.data.len() == 3
  }

  /*
    Returns data based on the lod level
  */
  pub fn lod(&self, level: usize) -> Vec<u8> {
    if level > self.get_depth() as usize {
      panic!("level can't be greater depth {}", level);
    }
    if level == self.get_depth() as usize {
      return self.data.clone();
    }

    let layer_mid = (self.layers[level + 1] - self.layers[level]) / 2;
    let last_index = self.layers[level] + layer_mid;
    self.data[0..last_index].to_vec()
  }

  fn get_descriptor_index(&self, layer: usize, local_index: usize) -> usize {
    let start_index = self.layers[layer];
    let layer_size = self.get_layer_size(layer);

    let branches = (layer_size / 2);
    start_index + branches + local_index
  }

  fn get_next_local_index(&self, layer: usize, local_index: usize, branch: u8) -> usize {
    let (layer_desc_start, layer_desc_size) = self.get_layer_desc_section(layer);

    let current_index = layer_desc_start + local_index;
    let current_desc = self.data[current_index];
    let mut index = get_bit_index(current_desc, branch);
    // let mut index = 5;

    if layer_desc_size == 0 {
      return index;
    }

    // Indexing through layer's descriptions is from right to left by default
    // To properly

    // println!("local_index {}", local_index);
    for i in 0..local_index {
      let desc_index = layer_desc_start + i;
      let desc = self.data[desc_index];
      let count = VoxelOctree::get_branch_count(desc);
    }
    index
  }

  /**
   * start_index, layer_size
   */
  fn get_layer_section(&self, layer: usize) -> (usize, usize) {
    if self.layer_section_cache.len() > 0 && self.layer_section_cache.len() > layer {
      return self.layer_section_cache[layer];
    }
    let mut start_index = 0;
    let mut layer_size = 0;
    if layer >= self.layers.len() {
      return (start_index, layer_size);
    }

    start_index = self.layers[layer];
    layer_size = self.get_layer_size(layer);
    (start_index, layer_size)
  }

  fn cache_layer_sections(&mut self) {
    for layer in 0..(self.get_depth() + 1) as usize {
      let (layer_start, layer_size) = self.get_layer_section(layer);
      self.layer_section_cache.push((layer_start, layer_size));
    }
  }

  fn get_layer_desc_section(&self, layer: usize) -> (usize, usize) {
    let start_index = self.layers[layer];
    let layer_size = self.get_layer_size(layer);
    let desc_size = layer_size / 2;
    (start_index + desc_size, desc_size)
  }

  fn get_layer_size(&self, layer: usize) -> usize {
    let next_layer = layer + 1;
    if next_layer >= self.layers.len() {
      return 0;
    }
    let start_index = self.layers[layer];
    let end_index = self.layers[next_layer];
    end_index - start_index
  }

  fn get_default_value(&self, layer: usize, local_index: usize) -> u8 {
    let (start, _) = self.get_layer_section(layer);
    let default_voxel_index = start + local_index;
    self.data[default_voxel_index]
  }

  /*
    Pseudocode:
      Split data between branches and voxel value
      Print branches as binary
      Print voxel value as normal u8
      Have to take note of default value
  */
  pub fn print_data(&self) {
    /*
      TODO:
        Split branches
          How to detect the data index to split?
    */

    // FIXME: Print as default for now
    println!("{:?}", self.data);
  }
}


fn check_out_of_bound_access(size: u32, x: u32, y: u32, z: u32) {
  if x >= size {
    panic!("x: {} cannot be greater than size: {}", x, size);
  }
  if y >= size {
    panic!("y: {} cannot be greater than size: {}", y, size);
  }
  if z >= size {
    panic!("z: {} cannot be greater than size: {}", z, size);
  }
}

fn get_bit_index(descriptor: u8, branch: u8) -> usize {
  if branch == 0 {
    panic!("Invalid branch value {}", branch);
  }

  let mut target_index = 0;
  let mut i = 0;
  for bit in 0..8 {
    let current_branch = 1 << bit;
    if descriptor & current_branch == current_branch {
      if current_branch == branch {
        // target_index = i;
        return i;
      }
      i += 1;
    }
  }
  target_index
}

pub fn get_index(positions: &Vec<[f32; 3]>, current_positions: &Vec<[f32; 3]>) -> u32 {
  let cur_index = if positions.len() == 0 {
    if current_positions.len() == 0 {
      0
    } else {
      current_positions.len() as u32
    }
  } else {
    (positions.len() + current_positions.len()) as u32
  };
  cur_index
}

fn calc_child(
  nodes: &mut Vec<Option<Node>>,
  index: &usize,
  current_depth: usize,
  default_value: u8,
  layer_details: &mut Vec<LayerDetail>,
  values: &mut Vec<u8>,
  mode: ParentValueType
) {
  let mut parent_default_value = 0;
  let mut children_values = Vec::new();
  let mut p_node = nodes.get(*index).unwrap().as_ref().unwrap().clone();
  for child_index in p_node.children.iter() {
    if *child_index > 0 {
      calc_child(
        nodes,
        child_index,
        current_depth + 1,
        default_value,
        layer_details,
        values,
        mode
      );

      let child = nodes.get(*child_index).unwrap().as_ref().unwrap();
      children_values.push(child.default_value);
    }
  }

  let is_leaf = p_node.values.len() != 0;
  let mut descriptor = p_node.descriptor;

  if is_leaf {
    parent_default_value = has_most_occurrence_value(default_value, &p_node.values, mode);

    let mut remove = 0;
    for (index, val) in p_node.values.iter().enumerate() {
      if *val == parent_default_value {
        remove += 1 << descriptor_with_branch_index(p_node.descriptor, index);
      } else {
        values.push(*val);
      }
    }

    descriptor &= !remove;

    p_node.default_value = parent_default_value;
    nodes[*index].insert(p_node);
  } else {
    parent_default_value = has_most_occurrence_value(default_value, &children_values, mode);

    p_node.default_value = parent_default_value;
    nodes[*index].insert(p_node);
  }

  // // FIXME: Just mimicking how set_voxel() should work, enable code above later
  // parent_default_value = default_value;
  // values.append(&mut p_node.values.clone());

  layer_details[current_depth]
    .default_values
    .push(parent_default_value);

  layer_details[current_depth].descriptors.push(descriptor);

  /*
    To get to the mapping:
      Every layer
      index = branch_index(from descriptor)

      local_layer_index = previous_branch_index(from descriptor)
      next_local_layer_index = layer_mappings[local_layer_index]
      data_index = branch_index(from descriptor) + layer_map_index
  */
  let descriptors = &layer_details[current_depth].descriptors;
  let first_valid_descriptor = descriptors.len() == 1;
  if first_valid_descriptor {
    layer_details[current_depth].mapping.push(0);

    if current_depth == layer_details.len() - 1 {
      let branches = VoxelOctree::get_branch_count(descriptor);
      layer_details[current_depth].mapping.push(branches);
    }
  } else {
    if !is_leaf {
      let mapping = &layer_details[current_depth].mapping;
      let last_map_index = mapping[mapping.len() - 1];

      let pre_index = descriptors.len() - 2;
      let pre_branches = VoxelOctree::get_branch_count(descriptors[pre_index]);

      let layer_index = last_map_index + pre_branches;

      layer_details[current_depth].mapping.push(layer_index);
    } else {
      let mapping = &layer_details[current_depth].mapping;
      let last_map_index = mapping[mapping.len() - 1];

      // println!("current_depth {} descriptors {:?}", current_depth, descriptors);
      let pre_index = descriptors.len() - 1;
      let pre_branches = VoxelOctree::get_branch_count(descriptors[pre_index]);

      let layer_index = last_map_index + pre_branches;

      layer_details[current_depth].mapping.push(layer_index);
    }
  }
}

fn process_leaf(index: &usize, layer_id: &u8, voxel_value: u8, nodes: &mut Vec<Option<Node>>) {
  let last_node_op = nodes.get_mut(*index).unwrap().as_mut();
  let last_node = last_node_op.unwrap();

  let branch = 1 << layer_id;
  let branch_already_set = last_node.descriptor & branch == branch;
  if !branch_already_set {
    last_node.descriptor = last_node.descriptor | branch;
  }

  let index = branch_index_reverse(last_node.descriptor, branch);
  last_node.values.insert(index, voxel_value);
}

fn process_branch(
  index: &usize,
  parent_index: &usize,
  layer_id: &u8,
  nodes: &mut Vec<Option<Node>>,
) {
  let c_node = nodes.get(*index).unwrap();
  if c_node.is_none() {
    nodes[*index].insert(Node::default());
  }

  let mut parent = nodes.get_mut(*parent_index).unwrap().as_mut().unwrap();
  parent.children[*layer_id as usize].clone_from(index);
  parent.descriptor = parent.descriptor | 0b_0000_0001_u8 << layer_id;
}

fn branch_index_reverse(descriptor: u8, branch: u8) -> usize {
  let mut index = 0;
  for bit in 0..8 {
    let tmp_branch = 1 << bit;
    if descriptor & tmp_branch == tmp_branch {
      if branch == tmp_branch {
        return index;
      }
      index += 1;
    }
  }
  panic!("branch {} not present in descriptor {}", branch, descriptor);
}

fn branch_index_reverse_in_descriptor(descriptor: u8, branch: u8) -> usize {
  let mut index = 0;
  for bit in 0..8 {
    let tmp_branch = 1 << bit;
    if descriptor & tmp_branch == tmp_branch {
      if branch == tmp_branch {
        return index;
      }
      index += 1;
    }
  }
  0
}

fn descriptor_with_branch_index(descriptor: u8, value_index: usize) -> usize {
  let mut has_value_index = 0;
  for bit in 0..8 {
    let tmp_branch = 1 << bit;
    if descriptor & tmp_branch == tmp_branch {
      if has_value_index == value_index {
        return bit;
      }
      has_value_index += 1;
    }
  }
  panic!("value_index doesn't exists {}", value_index);
}

fn has_most_occurrence_value(default_value: u8, values: &Vec<u8>, mode: ParentValueType) -> u8 {
  let mut highest = 0;
  let mut value = 0;
  if mode == ParentValueType::Lod {
    for val in values.iter() {
      let occurrences = values.iter().filter(|x| **x == *val).count();
      if occurrences > highest {
        highest = occurrences;
        value = *val;
      }
    }
  }

  if mode == ParentValueType::FillEmptyChildrenWithDefaultValue {
    let mut tmp_values = values.clone();
    let remaining = 8 - tmp_values.len();
    if remaining > 0 {
      for _ in 0..remaining {
        tmp_values.push(default_value);
      }
    }

    for val in tmp_values.iter() {
      let occurrences = tmp_values.iter().filter(|x| **x == *val).count();
      if occurrences > highest {
        highest = occurrences;
        value = *val;
      }
    }
  }
  if mode == ParentValueType::DefaultValue {
    value = default_value;
  }

  value
}

/** Convert node key Vec<u8> into u32 for faster HashMap key access */
fn get_num_key(key: &Vec<u8>) -> usize {
  if key.len() == 0 {
    return 0;
  }

  let mut index = 1;
  for (i, k) in key.iter().enumerate() {
    if i == 0 {
      index += *k as u32;
      continue;
    }
    let initial = (*k + 1) as u32;
    let place = i as u32;
    let next_value = initial * (8_i32.pow(place)) as u32;
    index += next_value;
  }
  index as usize
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_get_voxel() -> Result<(), String> {
    let default_value = 100;
    let data = vec![
      2,
      default_value,
      0b_1000_0001_u8,
      13,
      12,
      0b_0000_0001_u8,
      0b_1000_0001_u8,
      16,
      15,
      14,
    ];

    let octree = VoxelOctree::new_from_bytes(data);
    // println!("octree.data {:?}", octree.data);
    let size = octree.get_size();
    assert_eq!(4, size);
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          let expected_value = match (x, y, z) {
            (3, 2, 2) | (2, 3, 2) | (2, 2, 3) | (3, 3, 2) | (2, 3, 3) | (3, 2, 3) => 12,
            (1, 0, 0) | (0, 1, 0) | (0, 0, 1) | (1, 1, 0) | (0, 1, 1) | (1, 0, 1) | (1, 1, 1) => 13,
            (3, 3, 3) => 14,
            (2, 2, 2) => 15,
            (0, 0, 0) => 16,
            _ => default_value,
          };
          assert_eq!(
            expected_value,
            octree.get_voxel(x, y, z),
            "coordinate {} {} {}",
            x,
            y,
            z
          );
        }
      }
    }
    println!("test");
    Ok(())
  }

  #[test]
  fn test_set_voxel() -> Result<(), String> {
    let default_value = 0;
    let depth = 3;
    let data = vec![depth, default_value, 0b_0000_0000_u8];
    let mut octree = VoxelOctree::new_from_bytes(data);
    let size = octree.get_size();
    let mut new_value = 0;

    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };

          octree.set_voxel(x, y, z, new_value);
          assert_eq!(
            octree.get_voxel(x, y, z),
            new_value,
            "at pos: {:?}",
            (x, y, z)
          );
        }
      }
    }

    new_value = 0;
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };

          assert_eq!(
            octree.get_voxel(x, y, z),
            new_value,
            "at pos: {:?}",
            (x, y, z)
          );
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_set_voxel2() -> Result<(), String> {
    let default_value = 0;
    let depth = 3;
    let data = vec![depth, default_value, 0b_0000_0000_u8];
    let mut octree = VoxelOctree::new_from_bytes(data);
    let size = octree.get_size();
    let mut new_value = 0;

    let mid = 5;
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = 0;
          if y < mid {
            new_value = 1;
          }

          octree.set_voxel(x, y, z, new_value);
        }
      }
    }

    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = 0;
          if y < mid {
            new_value = 1;
          }

          assert_eq!(
            octree.get_voxel(x, y, z),
            new_value,
            "at pos: {:?}",
            (x, y, z)
          );
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_set_voxel_reverse_order() -> Result<(), String> {
    let default_value = 0;
    let mut octree = VoxelOctree::new(default_value, 4);

    let values = vec![[2, 2, 2, 14], [0, 0, 0, 10]];
    for val in values.iter() {
      octree.set_voxel(val[0], val[1], val[2], val[3] as u8);
    }

    for val in values.iter() {
      assert_eq!(
        val[3] as u8,
        octree.get_voxel(val[0], val[1], val[2]),
        "at pos: {:?}",
        (val[0], val[1], val[2])
      );
    }
    Ok(())
  }

  #[test]
  fn test_new_from_3d_array1() -> Result<(), String> {
    let default_value = 0;
    let depth = 3;
    let data = vec![depth, default_value, 0b_0000_0000_u8];
    let mut tmp_octree = VoxelOctree::new_from_bytes(data);
    let size = tmp_octree.get_size();
    let mut new_value = 0;

    let mut voxels = Vec::new();
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };

          voxels.push([x as u32, y as u32, z as u32, new_value as u32]);
        }
      }
    }

    let octree = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::DefaultValue);

    new_value = 0;
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };

          assert_eq!(
            octree.get_voxel(x, y, z),
            new_value,
            "at pos: {:?}",
            (x, y, z)
          );
        }
      }
    }

    Ok(())
  }

  #[test]
  fn test_new_from_3d_array_cube_chunk_depth3() -> Result<(), String> {
    let default_value = 0;
    let depth = 3;
    let data = vec![depth, default_value, 0b_0000_0000_u8];
    let mut octree_set_voxel = VoxelOctree::new_from_bytes(data);
    
    let size = octree_set_voxel.get_size();
    let start = 2;
    let end = size - 2;

    let mut voxels = Vec::new();
    for x in start..end {
      for y in start..end {
        for z in start..end {
          let mut value = 1;
          octree_set_voxel.set_voxel(x as u32, y as u32, z as u32, value as u8);
          voxels.push([x as u32, y as u32, z as u32, value as u32]);
        }
      }
    }
    let mut octree_new_from_3d_array = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::DefaultValue);

    println!("1 {:?}", octree_new_from_3d_array.data);
    println!("");
    println!("2 {:?}", octree_set_voxel.data);

    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          let mut value = 1;
          let val1 = octree_new_from_3d_array.get_voxel(x as u32, y as u32, z as u32);
          let val2 = octree_set_voxel.get_voxel(x as u32, y as u32, z as u32);

          assert_eq!(
            val1,
            val2,
            "at pos: {:?}",
            (x, y, z)
          );
        }
      }
    }

    // assert_eq!(octree_set_voxel.data.len(), octree_new_from_3d_array.data.len());
    // for (index, value) in octree_set_voxel.data.iter().enumerate() {
    //   assert_eq!(value, &octree_new_from_3d_array.data[index], "At index {}", index);
    // }


    Ok(())
  }

  #[test]
  fn test_new_from_3d_array_cube_chunk_depth4() -> Result<(), String> {
    let default_value = 0;
    let depth = 4;
    let data = vec![depth, default_value, 0b_0000_0000_u8];
    let mut octree_set_voxel = VoxelOctree::new_from_bytes(data);
    
    let size = octree_set_voxel.get_size();
    let start = 2;
    let end = size - 2;

    let mut voxels = Vec::new();
    let mut index = 0;
    'x: for x in start..end {
      for y in start..end {
        for z in start..end {
          let mut value = 1;
          octree_set_voxel.set_voxel(x as u32, y as u32, z as u32, value as u8);
          voxels.push([x as u32, y as u32, z as u32, value as u32]);

          index += 1;
          if index == 15 {
            break 'x;
          }
        }
      }
    }

    let mut octree_new_from_3d_array = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::DefaultValue);

    println!("1 {:?}", octree_new_from_3d_array.data);
    println!("");
    println!("2 {:?}", octree_set_voxel.data);

    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          let mut value = 1;
          let val1 = octree_new_from_3d_array.get_voxel(x as u32, y as u32, z as u32);
          let val2 = octree_set_voxel.get_voxel(x as u32, y as u32, z as u32);

          assert_eq!(
            val1,
            val2,
            "at pos: {:?}",
            (x, y, z)
          );
        }
      }
    }

    // assert_eq!(octree_set_voxel.data.len(), octree_new_from_3d_array.data.len());
    // for (index, value) in octree_set_voxel.data.iter().enumerate() {
    //   assert_eq!(value, &octree_new_from_3d_array.data[index], "At index {}", index);
    // }


    Ok(())
  }




  /* TODO: Have to update all the unit tests below */
  #[test]
  fn test_set_voxel_new_from_3d_array1() -> Result<(), String> {
    let default_value = 0;
    let depth = 2;
    let mut voxels = vec![[0, 0, 0, 0], [1, 0, 0, 0], [1, 1, 1, 10], [2, 2, 2, 20]];
    let mut voxels_map = HashMap::new();
    for voxel in voxels.iter() {
      voxels_map.insert([voxel[0], voxel[1], voxel[2]], voxel[3]);
    }

    let size = 2_i32.pow(depth);
    let octree = VoxelOctree::new_from_3d_array(default_value, depth as u8, &voxels, ParentValueType::Lod);
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          let value_op = voxels_map.get(&[x, y, z]);
          if value_op.is_some() {
            let value = *value_op.unwrap() as u8;
            assert_eq!(value, octree.get_voxel(x, y, z), "At {} {} {}", x, y, z);
          }
        }
      }
    }

    let expected = [2, 0, 129, 0, 0, 128, 1, 10, 20];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }
    Ok(())
  }

  #[test]
  fn test_set_voxel_new_from_3d_array2() -> Result<(), String> {
    let default_value = 14;
    let depth = 2;
    let mut voxels = Vec::new();

    let size = 2_i32.pow(depth);
    let mut new_value = 0;
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };
          voxels.push([x, y, z, new_value]);
        }
      }
    }

    let octree = VoxelOctree::new_from_3d_array(default_value, depth as u8, &voxels, ParentValueType::Lod);
    new_value = 0;
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          new_value = if new_value == 255 { 0 } else { new_value + 1 };
          assert_eq!(
            octree.get_voxel(x, y, z),
            new_value as u8,
            "At {} {} {}",
            x,
            y,
            z
          );
        }
      }
    }

    let expected = [
      2, 1, 255, 1, 33, 9, 41, 3, 35, 11, 43, 254, 254, 254, 254, 254, 254, 254, 254, 17, 5, 21, 2,
      18, 6, 22, 49, 37, 53, 34, 50, 38, 54, 25, 13, 29, 10, 26, 14, 30, 57, 45, 61, 42, 58, 46,
      62, 19, 7, 23, 4, 20, 8, 24, 51, 39, 55, 36, 52, 40, 56, 27, 15, 31, 12, 28, 16, 32, 59, 47,
      63, 44, 60, 48, 64,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }
    Ok(())
  }

  #[test]
  fn test_set_voxel_new_from_3d_array_set1() -> Result<(), String> {
    let default_value = 0;
    let depth = 2;
    let mut voxels = vec![[0, 0, 0, 1], [0, 0, 1, 1], [1, 1, 1, 10], [2, 2, 2, 20]];
    let mut voxels_map = HashMap::new();
    for voxel in voxels.iter() {
      voxels_map.insert([voxel[0], voxel[1], voxel[2]], voxel[3]);
    }

    let size = 2_i32.pow(depth);
    let octree = VoxelOctree::new_from_3d_array(default_value, depth as u8, &voxels, ParentValueType::Lod);
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          let value_op = voxels_map.get(&[x, y, z]);
          if value_op.is_some() {
            let value = *value_op.unwrap() as u8;
            assert_eq!(value, octree.get_voxel(x, y, z), "At {} {} {}", x, y, z);
          }
        }
      }
    }

    let expected = [2, 0, 129, 0, 0, 145, 1, 1, 1, 10, 20];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_set_voxel_new_from_3d_array_set2() -> Result<(), String> {
    let default_value = 0;
    let depth = 4;
    let mut voxels = vec![
      [0, 0, 0, 10],
      [1, 0, 0, 20],
      [1, 1, 0, 21],
      [2, 0, 0, 11],
      [4, 1, 0, 22],
      [4, 2, 1, 23],
      [4, 0, 0, 12],
      [0, 1, 8, 13],
      // [4, 2, 0, 14],
      // [4, 4, 0, 15]
    ];
    let mut voxels_map = HashMap::new();
    for voxel in voxels.iter() {
      voxels_map.insert([voxel[0], voxel[1], voxel[2]], voxel[3]);
    }

    let size = 2_i32.pow(depth);
    let octree = VoxelOctree::new_from_3d_array(default_value, depth as u8, &voxels, ParentValueType::Lod);
    for v in voxels.iter() {
      let x = v[0];
      let y = v[1];
      let z = v[2];

      let value_op = voxels_map.get(&[x, y, z]);
      if value_op.is_some() {
        let value = *value_op.unwrap() as u8;
        assert_eq!(octree.get_voxel(x, y, z), value, "At {} {} {}", x, y, z);
      }
    }

    let expected = [
      4, 0, 17, 0, 0, 3, 1, 0, 0, 0, 3, 5, 1, 0, 0, 0, 0, 0, 11, 1, 5, 16, 4, 10, 20, 21, 11, 12,
      22, 23, 13,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_set_voxel_new_from_3d_array_set_all() -> Result<(), String> {
    let default_value = 14;
    let depth = 4;
    let mut voxels = Vec::new();

    let size = 2_i32.pow(depth);
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          let mut voxel = default_value;
          if x % 2 == 0 {
            voxel = 1;
          }
          voxels.push([x, y, z, voxel]);
        }
      }
    }

    let octree = VoxelOctree::new_from_3d_array(default_value as u8, depth as u8, &voxels, ParentValueType::Lod);
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          let mut voxel = default_value;
          if x % 2 == 0 {
            voxel = 1;
          }
          assert_eq!(
            octree.get_voxel(x, y, z),
            voxel as u8,
            "At {} {} {}",
            x,
            y,
            z
          );
        }
      }
    }
    Ok(())
  }

  #[test]
  fn test_set_voxel_new_from_3d_array5() -> Result<(), String> {
    let default_value = 0;
    let depth = 5;
    let voxels = vec![
      [20, 20, 20, 18],
      [0, 0, 0, 10],
      [2, 0, 0, 11],
      [2, 2, 2, 12],
      [3, 3, 3, 13],
      [7, 0, 0, 14],
      [7, 7, 7, 15],
      [8, 0, 0, 16],
      [10, 0, 0, 17],
    ];
    let mut voxels_map = HashMap::new();
    for voxel in voxels.iter() {
      voxels_map.insert([voxel[0], voxel[1], voxel[2]], voxel[3]);
    }

    let size = 2_i32.pow(depth);
    let octree = VoxelOctree::new_from_3d_array(default_value, depth as u8, &voxels, ParentValueType::Lod);
    for x in 0..size as u32 {
      for y in 0..size as u32 {
        for z in 0..size as u32 {
          let value_op = voxels_map.get(&[x, y, z]);
          if value_op.is_some() {
            let value = *value_op.unwrap() as u8;
            assert_eq!(value, octree.get_voxel(x, y, z), "At {} {} {}", x, y, z);
          }
        }
      }
    }
    let expected = [
      5, 0, 129, 0, 0, 3, 1, 0, 0, 0, 131, 1, 128, 0, 0, 0, 0, 0, 131, 2, 128, 3, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 1, 129, 2, 128, 1, 1, 1, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }
    Ok(())
  }

  #[test]
  fn test_octree_new_from_3d_array_depth2() -> Result<(), String> {
    let default_value = 14;
    let depth = 2;
    let voxels = vec![[0, 0, 0, 14], [2, 2, 2, 15], [3, 3, 3, 16]];
    let octree = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::Lod);

    let expected = [2, 14, 129, 14, 15, 0, 128, 16];
    let expected = [2, 14, 129, 14, 14, 0, 129, 15, 16];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_octree_new_from_3d_array_depth3() -> Result<(), String> {
    let default_value = 0;
    let depth = 3;
    let voxels = vec![
      [0, 0, 0, 10],
      [2, 0, 0, 11],
      [2, 2, 2, 12],
      [3, 3, 3, 13],
      [7, 0, 0, 14],
      [7, 7, 7, 15],
    ];

    let octree = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::Lod);
    let expected = [
      3, 0, 131, 0, 0, 0, 131, 2, 128, 0, 0, 0, 0, 0, 1, 1, 129, 2, 128, 10, 11, 12, 13, 14, 15,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }
    Ok(())
  }

  #[test]
  fn test_octree_new_from_3d_array_depth4() -> Result<(), String> {
    let default_value = 0;
    let depth = 4;
    let voxels = vec![
      [0, 0, 0, 10],
      [2, 0, 0, 11],
      [2, 2, 2, 12],
      [3, 3, 3, 13],
      [7, 0, 0, 14],
      [7, 7, 7, 15],
      [8, 0, 0, 16],
      [10, 0, 0, 17],
    ];

    let octree = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::Lod);
    let expected = [
      4, 0, 3, 0, 0, 131, 1, 0, 0, 0, 0, 131, 2, 128, 3, 0, 0, 0, 0, 0, 0, 0, 1, 1, 129, 2, 128, 1,
      1, 10, 11, 12, 13, 14, 15, 16, 17,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_octree_new_from_3d_array_depth5() -> Result<(), String> {
    let default_value = 0;
    let depth = 5;
    let voxels = vec![
      [20, 20, 20, 18],
      [0, 0, 0, 10],
      [2, 0, 0, 11],
      [2, 2, 2, 12],
      [3, 3, 3, 13],
      [7, 0, 0, 14],
      [7, 7, 7, 15],
      [8, 0, 0, 16],
      [10, 0, 0, 17],
    ];

    let octree = VoxelOctree::new_from_3d_array(default_value, depth, &voxels, ParentValueType::Lod);
    let expected = [
      5, 0, 129, 0, 0, 3, 1, 0, 0, 0, 131, 1, 128, 0, 0, 0, 0, 0, 131, 2, 128, 3, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 1, 129, 2, 128, 1, 1, 1, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_octree_new_from_3d_array_using_random_value_set() -> Result<(), String> {
    let chunk_size = 16;
    let start = 0;
    let end_x = chunk_size;
    let end_y = chunk_size;
    let end_z = chunk_size;

    let mut data = Vec::new();
    for octree_x in start..end_x {
      for octree_y in start..end_y {
        for octree_z in start..end_z {
          let mut voxel = 0;
          if octree_x < 10 || octree_x < 10 {
            voxel = 1;
          }

          data.push([octree_x, octree_y, octree_z, voxel as u32]);
        }
      }
    }

    let depth = f32::sqrt(chunk_size as f32) as u8;
    let octree = VoxelOctree::new_from_3d_array(0, depth, &data, ParentValueType::Lod);

    for octree_x in start..end_x {
      for octree_y in start..end_y {
        for octree_z in start..end_z {
          let mut voxel = 0;
          if octree_x < 10 || octree_x < 10 {
            voxel = 1;
          }

          let saved_voxel = octree.get_voxel(octree_x, octree_y, octree_z);
          if voxel != saved_voxel {
            println!(
              "Not same {} {} {} {}",
              octree_x, octree_y, octree_z, saved_voxel
            );
          }
          assert_eq!(voxel, saved_voxel);
        }
      }
    }
    let expected = [
      4, 1, 255, 1, 1, 1, 1, 1, 1, 1, 1, 255, 255, 255, 255, 255, 255, 255, 255, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 255, 255,
      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
      255, 255, 255, 255, 255, 255, 255, 255, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0,
      1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1,
      0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_recalculate_start_layer_indices() -> Result<(), String> {
    let d = 9;
    let data = vec![3, d, 0b_0000_0000_u8];
    let mut octree = VoxelOctree::new_from_bytes(data);

    let expected = vec![1, 3];
    assert_eq!(octree.layers.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.layers[index], "At index {}", index);
    }

    let expected = vec![1, 3, 5];
    octree.data = vec![3, d, 1];
    octree.calculate_start_layer_indices();
    assert_eq!(octree.layers.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.layers[index], "At index {}", index);
    }

    octree.data = vec![3, d, 3, d, d, 1, 1, d, d, 1, 1, 1, 1];
    octree.calculate_start_layer_indices();
    let expected = vec![1, 3, 7, 11];
    assert_eq!(octree.layers.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.layers[index], "At index {}", index);
    }

    octree.data = vec![
      3, d, 7, d, d, d, 1, 3, 1, d, d, d, d, 1, 1, 1, 1, 1, 1, 1, 1,
    ];
    octree.calculate_start_layer_indices();
    let expected = vec![1, 3, 9, 17];
    assert_eq!(octree.layers.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.layers[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_get_next_local_index() -> Result<(), String> {
    let d = 9;
    let data = vec![3, d, 0b_0000_0000_u8];
    let mut octree = VoxelOctree::new_from_bytes(data);

    let data = vec![
      3, d, 7, d, d, d, 1, 7, 3, d, d, d, d, d, d, 1, 1, 1, 3, 1, 1, 7, 6, 5, 4, 3, 2, 1,
    ];
    octree = VoxelOctree::new_from_bytes(data);

    let result = octree.get_next_local_index(0, 0, 1);
    assert_eq!(result, 0);
    let result = octree.get_next_local_index(0, 0, 2);
    assert_eq!(result, 1);
    let result = octree.get_next_local_index(0, 0, 4);
    assert_eq!(result, 2);

    Ok(())
  }

  #[test]
  fn test_lod() -> Result<(), String> {
    let d = 9;
    let data = vec![
      3, d, 7, d, d, d, 1, 7, 3, d, d, d, d, d, d, 1, 1, 1, 3, 1, 1, 7, 6, 5, 4, 3, 2, 1,
    ];
    let mut octree = VoxelOctree::new_from_bytes(data.clone());

    let lod_0 = octree.lod(0);
    let expected = [3, d];
    assert_eq!(lod_0.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &lod_0[index], "At index {}", index);
    }

    let lod_1 = octree.lod(1);
    let expected = [3, d, 7, d, d, d];
    assert_eq!(lod_1.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &lod_1[index], "At index {}", index);
    }

    let lod_2 = octree.lod(2);
    let expected = [3, d, 7, d, d, d, 1, 7, 3, d, d, d, d, d, d];
    assert_eq!(lod_2.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &lod_2[index], "At index {}", index);
    }

    let lod_3 = octree.lod(3);
    let expected = data.clone();
    assert_eq!(lod_3.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &lod_3[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_lod_0_voxeloctree_get_voxel() -> Result<(), String> {
    let default = 9;
    let voxels = vec![[0, 0, 0, 10], [4, 0, 0, 0]];
    let octree_default = VoxelOctree::new_from_3d_array(9, 3, &voxels, ParentValueType::Lod);
    let octree = VoxelOctree::new_from_bytes(octree_default.lod(0));
    let expected = 10;
    let size = octree.get_size();
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          let result = octree.get_voxel(x, y, z);
          assert_eq!(default, result, "at {} {} {}", x, y, z);
        }
      }
    }

    let expected = [3, 9];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }
    Ok(())
  }

  #[test]
  fn test_lod_1_voxeloctree_get_voxel() -> Result<(), String> {
    let default = 9;
    let voxels = vec![[0, 0, 0, 10], [4, 0, 0, 0]];

    let default_octree = VoxelOctree::new_from_3d_array(default, 3, &voxels, ParentValueType::Lod);
    let octree = VoxelOctree::new_from_bytes(default_octree.lod(1));
    let size = octree.get_size();
    for x in 0..size {
      for y in 0..size {
        for z in 0..size {
          let result = octree.get_voxel(x, y, z);

          // if x >= 4 && y <= 3 && z <= 3 {
          //   let expected = 0;
          //   assert_eq!(result, expected, "at {} {} {}", x, y, z);
          // } else {
          //   let default = 10;
          //   assert_eq!(result, expected, "at {} {} {}", x, y, z);
          // }
        }
      }
    }

    let expected = [3, 9, 3, 9, 9];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }

    Ok(())
  }

  #[test]
  fn test_lod_2_voxeloctree_get_voxel() -> Result<(), String> {
    let default = 9;
    let voxels = vec![[0, 0, 0, 10], [4, 0, 0, 20], [6, 0, 0, 30]];
    let default_octree = VoxelOctree::new_from_3d_array(default, 3, &voxels, ParentValueType::Lod);
    let octree = VoxelOctree::new_from_bytes(default_octree.lod(2));

    let expected = [3, 9, 3, 9, 9, 1, 3, 9, 9, 9];
    assert_eq!(octree.data.len(), expected.len());
    for (index, value) in expected.iter().enumerate() {
      assert_eq!(value, &octree.data[index], "At index {}", index);
    }
    Ok(())
  }
}
