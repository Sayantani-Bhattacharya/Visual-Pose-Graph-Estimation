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

What does it optimize?
- The pose graph: a set of nodes (camera poses) and edges (relative pose constraints between those poses).

- Optimization: When you call the optimizer's optimize() method, it will adjust all the node poses to best satisfy all the relative pose constraints (edges) in a least-squares sense.

- Goal: Minimize the total error in the graph, which typically means making the estimated trajectory as consistent as possible with all the measured relative transformations (from visual odometry, loop closures, etc).


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



## Stereo Impl

Using rectified raw IR Imaging data:
Rectification here refers:
    >> Correct for lens distortions (e.g., barrel distortion)
    >> Align the left and right images so that corresponding points lie on the same     horizontal line (epipolar geometry)

The term "raw" in /image_rect_raw refers to the fact that:
    >> The image is not filtered, colorized, or depth-processed.
    >> It has been rectified, but remains a grayscale IR image suitable for depth estimation algorithms.

Why the IR Cameras Are Needed (To calculate depth)

RealSense stereo cameras (like D435, D455) use a stereo depth sensing pipeline:
    Two infrared cameras (left and right) capture the scene in IR light.
    They form a stereo pair and perform stereo matching to compute disparity.
    Disparity → Depth using triangulation + calibration parameters.
    Depth sensing is more robust than RGB-based depth estimation (e.g., from monocular depth or color stereo).

Internals of RealSense:
Component	Purpose	Needed for Depth?
Left IR camera	Stereo input	✅ Yes
Right IR camera	Stereo input	✅ Yes
IR projector	Add texture for matching	✅ (improves)
RGB camera	Color image capture	❌ No

## Loop closure techniques:

1. **Brute-Force Loop Closure**

### **Theory**
- For each new frame, compare its feature descriptors (e.g., SIFT, ORB) to those of all previous keyframes.
- Use a matcher (e.g., OpenCV’s `BFMatcher`) to find correspondences.
- If enough good matches are found (and pass geometric verification), a loop closure is detected.

### **Pros**
- Simple to implement.
- No need for pre-training or vocabulary.

### **Cons**
- **Slow:** Matching against all previous frames is computationally expensive, especially as the map grows.
- Not scalable for large datasets.
- May be less robust to appearance changes.

---

2. **Bag-of-Words (BoW) Loop Closure**


### **Theory**
- Extract features from each frame and quantize them into "visual words" using a pre-trained vocabulary (e.g., DBoW2/DBoW3).
- Represent each image as a histogram of visual words (BoW vector).
- For loop closure, compare the BoW vector of the current frame to those of previous frames using fast vector similarity (e.g., L1/L2/Chi-squared distance).
- Only if a candidate is similar enough, perform descriptor matching and geometric verification.

### **Pros**
- **Fast and scalable:** Only compare BoW vectors, not all descriptors.
- Efficient for large maps.
- More robust to appearance changes and viewpoint variations.
- Used in state-of-the-art SLAM systems (e.g., ORB-SLAM).

### **Cons**
- Requires a pre-trained vocabulary (can be generated offline).
- Slightly more complex to implement.

---

## **Summary**
- **Brute-force:** Simple, but slow and not scalable.
- **Bag-of-Words:** Fast, scalable, robust, and the standard for practical SLAM loop closure.

---

2. Brute-force


### Comparision:

+-------------+-------------------------------+----------------------------------------------------------+
| Aspect      | Brute-Force                   | Bag-of-Words (BoW)                                       |
+-------------+-------------------------------+----------------------------------------------------------+
| Matching    | All descriptors vs all        | Histogram (BoW vector) similarity, then descriptors      |
|             | descriptors                   |                                                          |
+-------------+-------------------------------+----------------------------------------------------------+
| Speed       | Slow (O(N))                   | Fast (O(1) for BoW, then O(M) for candidates)            |
+-------------+-------------------------------+----------------------------------------------------------+
| Scalability | Poor for large maps           | Excellent for large maps                                 |
+-------------+-------------------------------+----------------------------------------------------------+
| Robustness  | Lower                         | Higher (handles appearance change)                       |
+-------------+-------------------------------+----------------------------------------------------------+
| Implementation | Simple                     | Needs vocabulary, more complex                           |
+-------------+-------------------------------+----------------------------------------------------------+
| Used in     | Demos, small datasets         | Real SLAM systems (ORB-SLAM, etc.)                       |
+-------------+-------------------------------+----------------------------------------------------------+

