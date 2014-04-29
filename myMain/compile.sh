rm main
g++ main.cpp defMesh.cpp -I../leapSDK/include -o main -lGLU -lGL -lglut -L../Pinocchio/ -lpinocchio -L ../leapSDK/lib/x64/ -lLeap -Wl,-rpath,../leapSDK/lib/x64
