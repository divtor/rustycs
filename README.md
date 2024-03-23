# Rustycs
2D physics engine written in Rust.<br><br>
Usage:[Demo application using macroquad](https://github.com/divtor/rustycs-macroquad-demo)

## Setups
Rust:
* [Installing Rust](https://doc.rust-lang.org/book/ch01-00-getting-started.html)

VSC:
* [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer) - Improved Rust language support for VSC.
* [CodeLLDB](https://marketplace.visualstudio.com/items?itemName=vadimcn.vscode-lldb) - Native debugger powered by LLDB.
* [crates](https://marketplace.visualstudio.com/items?itemName=serayuzgur.crates) - Dependency manager for Cargo.

## Limitations and potential future features
Body Types:
* Supported:
    * Circle
    * Axis Aligned Bounding Boxes (AABB)
    * Oriented Bounding Boxes (OBB) (modelled as Polygons)
    * Convex Polygons

* Not supported:
    * Concave Polygons
        * The engine checks for concave polygons and panics when it detects one.
        * But if the polygon edges intersect one another the polygons do not get detected as concave even though they are, but this is easy to detect visually and should not be done.

The engine uses a "homemade" collision detection approach, where each shape combination is checked explicitly. 

Most parts of the engine are optimized to some extend, both memory and runtime wise. 
Nevertheless, there still is much room for possible improvements and optimizations, specifically:
* Utilizing multiple cores in the collision pipeline.
* Using a QuadTree for the collision broad phase.
* Restricting world bounds; currently unlimited (f32::MAX)
* Implementing continuos collision detection (CCD).
* ...

As this is a passion project, the engine has no implementation timeline etc.
Future functionality will be add whenever I feel like it.

## Some sources and inspirations
* [Box2D-lite](https://github.com/erincatto/box2d-lite)
* [Impulse Engine](https://github.com/RandyGaul/ImpulseEngine)
* [Collision Response by Chris Hecker](http://www.chrishecker.com/images/e/e7/Gdmphys3.pdf)
* and various other articles & papers on physics engines!
