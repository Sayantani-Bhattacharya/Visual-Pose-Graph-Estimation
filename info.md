# Libraries:

1. GTSAM (Georgia Tech Smoothing and Mapping)

What it does:
GTSAM is used to solve problems in robot perception and navigation — like SLAM (Simultaneous Localization and Mapping).
It helps a robot understand where it is and build a map of its environment by combining sensor data (e.g., cameras, IMUs).

How it works:

    It builds a graph where each node is a variable (like robot pose, landmark position), and each edge is a constraint (like a measurement).

    Then it uses optimization to find the most likely values of those variables.

2. g2o (General Graph Optimization)

What it does:
g2o is also used for graph-based optimization problems, especially in SLAM and bundle adjustment.

How it works:

    Like GTSAM, it models variables and constraints as a graph.

    It is fast and good for large graphs, like those in visual SLAM or 3D reconstruction.

    It minimizes errors between expected and measured data (like positions, landmarks).


3. Ceres Solver

What it does:
Ceres is a general-purpose optimization library.
It solves non-linear least squares problems, used in things like camera calibration, bundle adjustment, and curve fitting.

How it works:

    You give it a function (like an error or cost), and it tweaks the inputs to minimize the error.

    It’s widely used in computer vision (e.g., Google uses it in their mapping software).


# Algorithms:

## Feature extractions: 

1. SIFT: Scale-Invariant Feature Transform
>> Identify keypoints in a frame and define descriptor vector for them.
>> Gausian blur over images.
>> Substact
>> Create histogram


2. ORB
3. SURF
4. AKAZE





## Future improvements:


1. For stereo camera, we can use stereo geometry for pose estimation, which is more robust than monocular.

Current Impl: left-right fused (rectified) image for feature extraction and pose estimation, but you will lose the depth (3D) information that stereo geometry provides. 

Using Fused (Monocular) Image
    Pros:
    Simpler pipeline (treat as monocular SLAM/VO).
    Only need to extract and match features from one image per frame.
    Cons:
    You cannot triangulate 3D points directly from a single image.
    Pose estimation will be up-to-scale (scale ambiguity).
    You lose the robustness and metric scale that stereo provides.
Using Stereo (Left & Right) Images
    Pros:
    You can triangulate 3D points using disparity between left and right images.
    You get metric scale and more robust pose estimation.
    Better for real-world navigation and mapping.
    Cons:
    Slightly more complex pipeline (need to match left-right features and triangulate).