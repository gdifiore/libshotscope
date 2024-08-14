# libshotscope
`libshotscope` is a C++ library designed to simulate golf ball trajectories based on initial conditions like velocity, atmospheric data, and more. It provides easy-to-use functions to visualize or calculate the ball’s flight path and landing point.

Much of the math here is based on [work done](http://baseball.physics.illinois.edu/trajectory-calculator-golf.html) by Prof. Alan M. Nathan at the  University of Illinois Urbana-Champaign. I turned it into a C++ library and incorporated tests.

## Build
```bash
git clone https://github.com/gdifiore/libshotscope.git

cd libshotscope

chmod +x build.sh

./build.sh
```

## docs

[How to use](/docs/how.md) `libshotscope`

[Why/Where to use](/docs/why.md) `libshotscope`