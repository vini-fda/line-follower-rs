use std::collections::{HashMap, HashSet};

use linefollower_core::geometry::closed_path::ClosedPath;
use linefollower_core::geometry::track::Track;
use linefollower_core::{geometry::closed_path::SubPath, utils::traits::Float};
use mint::Point2;
use petgraph::prelude::DiGraph;
use petgraph::stable_graph::NodeIndex;
use petgraph::visit::EdgeRef;

pub type CurveGraph = DiGraph<Point2<f32>, SubPath<f64>>;

pub trait AddSubPath<F>
where
    F: Float,
{
    fn add_subpath(&mut self, subpath: SubPath<F>);
}

impl AddSubPath<f64> for CurveGraph {
    fn add_subpath(&mut self, subpath: SubPath<f64>) {
        let p0: Point2<f32> = subpath.first_point().cast::<f32>().into();
        let p1: Point2<f32> = subpath.last_point().cast::<f32>().into();
        const MIN_DISTANCE_SQR: f32 = 1e-10;
        let mut i0 = None;
        let mut i1 = None;
        // before adding the nodes, check if they are close enough to existing nodes
        for node in self.node_indices() {
            let p = self[node];
            fn distance_sqr(p0: Point2<f32>, p1: Point2<f32>) -> f32 {
                (p0.x - p1.x).powi(2) + (p0.y - p1.y).powi(2)
            }
            if distance_sqr(p, p0) <= MIN_DISTANCE_SQR {
                i0 = Some(node);
            }
            if distance_sqr(p, p1) <= MIN_DISTANCE_SQR {
                i1 = Some(node);
            }
        }
        // add nodes if they don't exist
        let i0 = i0.unwrap_or_else(|| self.add_node(p0));
        let i1 = i1.unwrap_or_else(|| self.add_node(p1));
        // add the edge
        self.add_edge(i0, i1, subpath);
    }
}

pub trait ValidTrack {
    fn valid_track(&self, node_indices: &[NodeIndex]) -> Option<ClosedPath<f64>>;
}

impl ValidTrack for CurveGraph {
    /// A valid track is a closed path that has exactly
    /// one outgoing edge per node.
    ///
    /// This method checks if the nodes given by `node_indices` forms a valid track
    /// and, if it does, it returns the corresponding closed path.
    fn valid_track(&self, node_indices: &[NodeIndex]) -> Option<ClosedPath<f64>> {
        if node_indices.len() < 2 {
            return None;
        }
        let first = node_indices[0];
        let mut subpaths = Vec::with_capacity(node_indices.len());
        let mut visited = HashSet::new();
        let mut next_node = first;
        loop {
            let node = next_node;
            visited.insert(node);
            if self.edges(node).count() != 1 {
                // a valid track has exactly one outgoing edge per node
                return None;
            }
            let edge = self.edges(node).next().unwrap();
            next_node = edge.target();
            if !node_indices.contains(&next_node) {
                // the next node is not part of the track
                return None;
            }
            // add the subpath to the track
            subpaths.push(self[edge.id()].clone());
            // as soon as it goes back to the first node...
            if next_node == first {
                // ...check if all nodes have been visited
                if visited.len() == node_indices.len() {
                    return Some(ClosedPath::new(subpaths));
                } else {
                    return None;
                }
            }
        }
    }
}
