use super::{line_tool::{LinePathTool, LineToolState}, free_tool::FreeTool, paintable::Paintable};
pub trait Tool: Paintable {
    //fn clicked(self) -> Tool;
}


impl Tool for FreeTool {
    
}
impl<S: LineToolState> Tool for LinePathTool<S> where LinePathTool<S>: Paintable {}