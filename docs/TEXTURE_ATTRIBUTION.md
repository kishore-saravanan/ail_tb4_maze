# Realistic scene materials

Downloaded original 1K JPG surface maps from **ambientCG**, licensed CC0. The simulation uses their albedo, OpenGL normal and roughness maps; native mesh UVs preserve physical texture scale on long walls. Assets are stored locally, so launch does not fetch network resources.

- Brick: [Bricks005](https://ambientcg.com/view?id=Bricks005), `Bricks005_1K-JPG.zip`.
- Soil: [Ground037](https://ambientcg.com/view?id=Ground037), `Ground037_1K-JPG.zip`.
- Green grass / hedge surface: [Grass001](https://ambientcg.com/view?id=Grass001), `Grass001_1K-JPG.zip`.
- [ambientCG license](https://docs.ambientcg.com/license/): CC0, including redistribution and commercial use.

The source files are `src/ail_tb4_maze/assets/textures/`. Scene meshes are generated locally with physical UV tiling. Green obstacle surfaces are combined with rounded foliage clusters to create shrub/hedge forms. This is a textured Gazebo scene, not a photorealistic offline render; collision geometry remains simple, deterministic and robot-traversable.

The official maze preset still derives from the Apache-2.0 TurtleBot 4 maze. Random-grid layouts are optional new variants explicitly requested by the user after the original infrastructure phase. They do not use or include a robot navigation algorithm.

The user preferred the earlier green/mossy ground over bare brown soil, so Ground037 is the final ground material. The briefly tested brown Ground048 textures are retained as an unused optional asset.
