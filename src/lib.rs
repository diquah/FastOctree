use std::{cell::RefCell, rc::Rc};

use fixed_typed_arena::ManuallyDropArena;
pub use glam::IVec3;

pub trait Volume {
    fn min(&self) -> IVec3;

    fn max(&self) -> IVec3;
}

pub struct Octree<'t, const R: usize> {
    root_cell: &'t mut OctreeCell<'t, R>,
    arena: Rc<RefCell<ManuallyDropArena<OctreeCell<'t, R>>>>,
}

impl<'t, const R: usize> Octree<'t, R> {
    pub fn new<T>(volumes: &'t Vec<&'t dyn Volume>) -> Self {
        let mut min_bound: glam::IVec3 = volumes.iter().fold(IVec3::ZERO, |a, b| a.min(b.min()));
        let mut max_bound: glam::IVec3 = volumes.iter().fold(IVec3::ZERO, |a, b| a.max(b.max()));

        debug_assert_ne!(
            min_bound, max_bound,
            "Volumes have no span (Octree has volume of 0)."
        );

        if min_bound.x % 2 != 0 {
            min_bound.x -= 1
        }
        if min_bound.y % 2 != 0 {
            min_bound.y -= 1
        }
        if min_bound.z % 2 != 0 {
            min_bound.z -= 1
        }

        if max_bound.x % 2 != 0 {
            max_bound.x += 1
        }
        if max_bound.y % 2 != 0 {
            max_bound.y += 1
        }
        if max_bound.z % 2 != 0 {
            max_bound.z += 1
        }

        let arena = Rc::new(RefCell::new(ManuallyDropArena::new()));
        let root_cell =
            OctreeCell::build_recursively(arena.clone(), min_bound, max_bound, &volumes);

        Self { root_cell, arena }
    }
}

impl<'t, const R: usize> Drop for Octree<'t, R> {
    fn drop(&mut self) {
        unsafe { self.arena.borrow_mut().manually_drop() };
    }
}

enum OctreeCell<'c, const R: usize> {
    FilledCell(FilledOctreeCell<'c, R>),
    EmptyCell(EmptyOctreeCell<'c, R>),
    EmptyChildlessCell,
}

impl<'c, const R: usize> OctreeCell<'c, R> {
    #[rustfmt::skip]
    fn build_recursively(
        arena: Rc<RefCell<ManuallyDropArena<OctreeCell<'c, R>>>>,
        min_bound: IVec3,
        max_bound: IVec3,
        volumes: &'c [&'c dyn Volume],
    ) -> &'c mut OctreeCell<'c, R> {
        let mut volumes_inside_cell: Vec<Option<&dyn Volume>> = volumes
            .iter()
            .filter_map(|v| {
                let bmax = v.max();
                let bmin = v.min();

                if min_bound.x <= bmax.x
                    && max_bound.x >= bmin.x
                    && min_bound.y <= bmax.y
                    && max_bound.y >= bmin.y
                    && min_bound.z <= bmax.z
                    && max_bound.z >= bmin.z
                {
                    Some(*v)
                } else {
                    None
                }
            })
            .map(|v| Some(v))
            .collect();

        if volumes_inside_cell.len() >= R {
            let midpoint = (max_bound + min_bound).div_euclid(IVec3::splat(2));

            let mut subregions_min = [IVec3::ZERO; 8];
            let mut subregions_max = [IVec3::ZERO; 8];

            for i in 0..8 {
                let corner_offset = IVec3::new(i & 1, (i >> 1) & 1, (i >> 2) & 1);
                subregions_min[i as usize] = min_bound + corner_offset * (midpoint - max_bound);
                subregions_max[i as usize] = midpoint + corner_offset * (midpoint - max_bound);
            }

            let a = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[0],
                subregions_max[0],
                volumes,
            );

            let b = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[1],
                subregions_max[1],
                volumes,
            );

            let c = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[2],
                subregions_max[2],
                volumes,
            );

            let d = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[3],
                subregions_max[3],
                volumes,
            );

            let e = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[4],
                subregions_max[4],
                volumes,
            );

            let f = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[5],
                subregions_max[5],
                volumes,
            );

            let g = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[6],
                subregions_max[6],
                volumes,
            );

            let h = OctreeCell::build_recursively(
                arena.clone(),
                subregions_min[7],
                subregions_max[7],
                volumes,
            );

            arena
                .borrow_mut()
                .alloc(OctreeCell::EmptyCell(EmptyOctreeCell {
                    a, b, c, d, e, f, g, h
                }))
        } else if volumes_inside_cell.len() == 0 {
            arena.borrow_mut().alloc(OctreeCell::EmptyChildlessCell)
        } else {
            volumes_inside_cell.resize_with(R, || None);
            arena
                .borrow_mut()
                .alloc(OctreeCell::FilledCell(FilledOctreeCell(
                    volumes_inside_cell.try_into().unwrap_or([None; R]),
                )))
        }
    }
}

struct EmptyOctreeCell<'c, const R: usize> {
    a: &'c OctreeCell<'c, R>,
    b: &'c OctreeCell<'c, R>,
    c: &'c OctreeCell<'c, R>,
    d: &'c OctreeCell<'c, R>,
    e: &'c OctreeCell<'c, R>,
    f: &'c OctreeCell<'c, R>,
    g: &'c OctreeCell<'c, R>,
    h: &'c OctreeCell<'c, R>,
}

struct FilledOctreeCell<'c, const R: usize>([Option<&'c dyn Volume>; R]);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(true, true);
    }
}
