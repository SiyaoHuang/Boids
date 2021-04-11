# Boids

A little experiment with boids in Unity:
https://www.youtube.com/watch?v=bqtqltqcQhw

For a much better implemented and performant version in Unity, have a look at the ECS sample project:
https://github.com/Unity-Technologies/EntityComponentSystemSamples/tree/master/ECSSamples/Assets/Advanced/Boids/Scripts

![Boids](https://i.imgur.com/Q1E488u.png)

# Comment by Siyao:

This is an extremely interesting project! It reminds me of visiting aquarium when I was young. 

It works well when there are 350 boids. But when I scale it to 3500 boids, my computer (NVDIA MX150) can only run around 3 fps. I optimized it using cast command and job system. It can run around 30 fps with 3500 boids on my computer now. 

There are still ways to optimize it. So far, it looks great to me.
