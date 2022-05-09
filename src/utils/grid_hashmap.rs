use hashbrown::HashMap;

use crate::{chunk::world_pos_to_key};

/*
  TODO
    Create a 3-dimensional grid to distribute access to HashMaps
  Implementation:
    Need to have grid size
    Setup unit test
    Setup benchmark
    Make this generic?
*/

pub struct GridHashMap<V> {
  maps: HashMap<[i64; 3], HashMap<[i64; 4], V>>,
  pub size: u32,
}

impl<V> GridHashMap<V> {
  pub fn get(&self, key: &[i64; 4]) -> Option<&V> {
    let key3 = [key[0], key[1], key[2]];
    let grid = world_pos_to_key(&key3, self.size);
    let map_op = self.maps.get(&grid);
    if map_op.is_none() {
      return None;
    }

    let local_key = get_local_key(&key3, self.size);
    let local_key4 = [local_key[0], local_key[1], local_key[2], key[3]];
    map_op.unwrap().get(&local_key4)
  }

  pub fn insert(&mut self, key: [i64; 4], value: V) {
    let key3 = [key[0], key[1], key[2]];
    let grid = world_pos_to_key(&key3, self.size);
    let mut map_op = self.maps.get(&grid);

    let local_key = get_local_key(&key3, self.size);
    let local_key4 = [local_key[0], local_key[1], local_key[2], key[3]];
    if map_op.is_none() {
      let mut new_map = HashMap::new();
      new_map.insert(local_key4, value);
      self.maps.insert(grid, new_map);
    } else {
      self.maps.get_mut(&grid).unwrap().insert(local_key4, value);
    }
  }

  pub fn contains_key(&self, key: &[i64; 4]) -> bool {
    self.get(key).is_some()
  }

  pub fn len(&self) -> usize {
    // self.maps.len()
    let mut len = 0;
    for (key, val) in self.maps.iter() {
      len += val.len();
    }
    len
  }
}

impl<V> Default for GridHashMap<V> {
  fn default() -> Self {
    GridHashMap {
      maps: HashMap::<[i64; 3], HashMap<[i64; 4], V>>::new(),
      size: 32,
    }
  }
}

/**
 Needed for negative key to identify local key
*/
fn get_local_key(&key: &[i64; 3], size: u32) -> [i64; 3] {
  let sizei64 = size as i64;
  let rel_x = key[0] % sizei64;
  let rel_y = key[1] % sizei64;
  let rel_z = key[2] % sizei64;

  let mut local_x = rel_x;
  if rel_x < 0 {
    local_x += sizei64;
  }
  let mut local_y = rel_y;
  if rel_y < 0 {
    local_y += sizei64;
  }
  let mut local_z = rel_z;
  if local_z < 0 {
    local_z += sizei64;
  }

  [local_x, local_y, local_z]
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_get_local_key() -> Result<(), String> {
    let grid_hashmap = GridHashMap::<bool>::default();
    let size = grid_hashmap.size;

    let key = [0, 0, 0];
    let local_key = get_local_key(&key, size);
    assert_eq!(local_key, [0, 0, 0]);

    let key = [1, 7, 8];
    let local_key = get_local_key(&key, size);
    assert_eq!(local_key, [1, 7, 0]);

    let key = [16, 24, 32];
    let local_key = get_local_key(&key, size);
    assert_eq!(local_key, [0, 0, 0]);

    let key = [-1, -7, -8];
    let local_key = get_local_key(&key, size);
    assert_eq!(local_key, [7, 1, 0]);

    let key = [-16, -24, -32];
    let local_key = get_local_key(&key, size);
    assert_eq!(local_key, [0, 0, 0]);

    let key = [-9, -17, -25];
    let local_key = get_local_key(&key, size);
    assert_eq!(local_key, [7, 7, 7]);

    Ok(())
  }

  #[test]
  fn test_get_chunk() -> Result<(), String> {
    let mut grid_hashmap = GridHashMap::default();
    let size = grid_hashmap.size;

    let keys = vec![[0, 0, 0, 0], [1, 0, 0, 0], [2, 0, 0, 0], [3, 0, 0, 0]];

    for key in keys.iter() {
      let chunk_op = grid_hashmap.get(key);
      assert!(
        chunk_op.is_none(),
        "Error chunk {:?} should not exists",
        key
      );
      assert!(
        !grid_hashmap.contains_key(key),
        "Error chunk {:?} should not exists",
        key
      );

      grid_hashmap.insert(*key, Chunk::default());
      let chunk_op = grid_hashmap.get(key);
      assert!(chunk_op.is_some(), "Error chunk {:?} should exists", key);
      assert!(
        grid_hashmap.contains_key(key),
        "Error chunk {:?} should exists",
        key
      );
    }

    Ok(())
  }

  /*
    TODO
      Add more unit tests later
  */
}
