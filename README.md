## The Finite Element Experience

This project is an effort to demonstrate the capabilities of the finite element method in 3D. The project is written mainly in C++ and OpenGL trough Magnum.

To build the project you need Magnum installed on your system. Magnum is split into a number of separate modules. Pre-built packages can be found for Arch but I've found building from source to be a more flexible approach here. Magnum also supports WebGL as the rendering backend and WASM via Emscripten.  
TODO:
Instructions for building the WASM version

Here is what is needed for the desktop build:

SDL2 (prebuilt or from source)  
https://github.com/mosra/corrade.git  
https://github.com/mosra/magnum.git  
https://github.com/mosra/magnum-plugins.git  
https://github.com/mosra/magnum-extras.git

This project has been developed against the "v2019.01" branch of the above repositories.

There is also a magnum-examples repository that might be worth looking into. Follow Magnum's instructions [here](https://doc.magnum.graphics/magnum/getting-started.html). "build.sh" shows some CMake flags that can be used for building. YES they need to be installed into /usr. I tried bundling with CMake's ExternalPackage_Add but I couldn't make it work due to how GLFW was handled.

Before the first build you also need a mesh file. See instructions below. After all dependencies are in place this project can be built the common way:

```
mkdir build
cd build
cmake ..
make -j
```

#### Mesh generation
Mesh generation is out of the scope for this project. Two MATLAB scripts are included for generating and saving tetrahedron meshes in a simple ASCII format. In order to have this project also WASM ready, the files are built into the final binary. This is specified in "src/resources.conf". You can generate a 3D mesh out of "cube.stl" and move the resulting file to src/.

GLSL shaders are written in separate "\*.vert" and "\*.frag" files and are included using the same mechanism as the mesh files. There are also a number of html files in src that can be used for the WASM build.

TODO:
- The mesh file also needs to include information about the boundary nodes
  - meshToPet works properly for matlab <= R2016a, in R2016b and newer its kind of broken (_no properties for e_).  
  - The [documentation](https://www.mathworks.com/help/pde/ug/pde.femesh.meshtopet.html) for meshToPet states its a legacy workflow and might not work
- The actual solver
