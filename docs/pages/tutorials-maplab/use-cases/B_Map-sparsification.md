## Map sparsification: make your mapping more efficient

Maplab offers several algorithms designed to reduce the computational/memory requirements of the map. That's particularly important when you would like to cover large areas or deal with a large number of mapping sessions.

Let's start with an initial map - one of the maps of a multi-floor dataset:
![initial](images/sparsification/init.png)

### Keyframing
To reduce the number of keyframes in the system (and, as a result, also remove some landmarks), you can use the keyframing command:
```
kfh
```
Keyframing is based on multiple criteria: 
* maximum distance: if translation > threshold then adds a keyframe,
* maximum rotation: if rotation > threshold then adds a keyframe,
* number of keyframes: forces keyframes at every n-th keyframe,
* number of coobserved landmarks: adds a keyframe if the landmarks co-observed by two frames is smaller than a threshold.

Those parameters can be adapted using flags:
* ``kf_distance_threshold_m``, default 0.75
* ``kf_rotation_threshold_deg``, default 20
* ``kf_every_nth_vertex``, default 10
* ``kf_min_shared_landmarks_obs``, default 40

The resulting map should contain significantly fewer keyframes:
![kfh](images/sparsification/kfh.png)

### Map summarization
Map summarization removes the landmarks using an Integer Linear Programming optimization, using ``lpsolve`` solver. It tries to keep the landmarks that are most often observed while also maintaining a good coverage over the entire area. 

To summarize to maps and only keep 5000 best landmarks, you can call:
```
lsparsify --num_landmarks_to_keep 5000
```

This reduces the number of landmarks and reduces the size of the map:
![lsparsify](images/sparsification/lsparsify.png)

### Exporting a localization summary map
Maplab introduces a concept of a compact localization summary map - a minimal map representation that our loopclosure algorithm can operate on. Basically, it consists of landmark 3D positions, observation descriptors and a covisibility graph.

To export a localization summary map that can be used by ROVIOLI, just use the command:
```
generate_summary_map_and_save_to_disk --summary_map_save_path your_summary_map
```
