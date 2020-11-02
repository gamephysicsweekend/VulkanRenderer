# VulkanRenderer

This is a very basic renderer that is meant to be used in conjuction with the [Game Physics In One Weekend book series](https://gamephysicsweekend.github.io/).

It is currently only targeted for Windows 10 and Visual Studio 2019.

When you get the project to build and run, you should see a small sphere on top of a large sphere.  This is sort of the hello world of the book series.

![Hello World](https://github.com/gamephysicsweekend/VulkanRenderer/blob/main/data/images/helloworld.jpg?raw=true)

## Usage

Scene.cpp is where you should begin writing the code from the book series.  And sprinkled throughout the rest of the code will be comments:

```
// TODO: Add code here
```

This is where you would fill in the code snippets from the texts.

## Controls

The controls for the renderer are very basic.

```
Mouse look to rotate the camera around the origin.
Scroll to zoom.
"T" to pause and unpause time.
"R" to reset the scene.
"Y" to step the simulation by a single frame.
```


## Vulkan Resources

Although this "renderer" uses Vulkan, it is not intended as a resource for learning it.  Instead, I recommend the following:

[https://vulkan-tutorial.com/](https://vulkan-tutorial.com/)

[https://www.fasterthan.life/blog/2017/7/11/i-am-graphics-and-so-can-you-part-1](https://www.fasterthan.life/blog/2017/7/11/i-am-graphics-and-so-can-you-part-1)

[https://github.com/SaschaWillems/Vulkan](https://github.com/SaschaWillems/Vulkan)

[https://www.lunarg.com/vulkan-sdk/](https://www.lunarg.com/vulkan-sdk/)