A sort of tutorial on 3d graphics where various 3d graphics algorithms are implemented.
Algorithms are implemented in their crude forms that can be understood easily.

Sorry if the code doesnot follow good coding / design practices.

1. `g++ -o output file_name.cpp -lSDL2`
2. `./output`

Convert to Javascript using EMCSCRIPTEN
1. `emcc 1_basic_3d_setup.cpp -s USE_SDL=2 -o basic.html`

2. Serve output in browser `python -m http.server 8080 --bind 127.0.0.1 --directory .`