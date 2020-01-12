use crate::{Process};

/// The observer is an iterator over a pair of processes, one that acts as a 
/// source (`Src`) and the other that acts as a destination (`Dst`). This serves
/// as an abstraction for hooking up a test signal generator to a device under 
/// test, and allows you collect observations on processes for later analysis. 
pub struct Observer <Src, Dst> 
where 
    Dst: Process + Clone, 
    Src: Process<Input = (), Output = Dst::Input>
{
    src: Option<Src>, 
    dst: Option<Dst>, 
}

impl<S,D> Observer<S,D> 
where D: Process + Clone, 
      S: Process<Input = (), Output = D::Input>  {
    pub fn new(src:S, dst:D) -> Self {
        Self { src: Some(src), dst : Some(dst) }
    }
}

impl<S,D> Iterator for Observer <S, D>
where D: Process + Clone, 
      S: Process<Input = (), Output = D::Input> {
    type Item = (D, D::Output); 
    fn next(&mut self) -> Option<Self::Item> {
        let src = self.src.take()?.evolve(());
        let dst = self.dst.take()?.evolve(src.observe());
        let out = dst.observe();
        self.src = Some(src); 
        self.dst = Some(dst.clone()); 
        Some((dst, out))
    }
}
