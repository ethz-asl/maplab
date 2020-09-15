## Expressing frame transformations in code

We will use the [minkindr library](https://github.com/ethz-asl/minkindr) to represent robot poses and we will follow the notational conventions both in documentation and in code.

The diagrams below specify our notational conventions for vector quantities, rotations, transformations, and points. A coding example with coding standards is at the bottom of the page.

![Vector quantities](https://cloud.githubusercontent.com/assets/1216052/3073884/ca425992-e311-11e3-8803-2c68fa89cf27.png)


![Rotation matrices](https://cloud.githubusercontent.com/assets/1216052/3073895/b88eb672-e312-11e3-8c75-507227003371.png)

![Transformations and points](https://cloud.githubusercontent.com/assets/1216052/3073906/f791dcae-e313-11e3-86ce-0b6a99a736da.png)

```c++
///
/// \brief A frame transformation example appending an odometric transform onto a robot pose
///
/// Coordinate frames:
///   - W : the world coordinate frame
///   - B : the robot body coordinate frame at time k
///
/// @param T_W_Bk  The robot pose at time k.
/// @param T_Bk_Bkp1 The odometric transform from odometry between time k and time k+1
/// @param out_T_W_Bkp1 The robot pose at time k+1
void frameTransformationExample(const Transformation& T_W_Bk, 
                                const Transformation& T_Bk_Bkp1, 
                                Transformation* out_T_W_Bkp1) {
//
}
```

Because we are using capital letters for the frame tags, we will reserve `R` and `T` as special values that *should not* be used as frame tags. `R` will represent rotation matrices (`R_A_B`) and T will represent transformation matrices (`T_A_B`).
