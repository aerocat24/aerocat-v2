# STL files and assembly

This folder contains the STL files for printing the mechanical parts of the
small AeroCat V2 catamaran. Reference images for each part are available in the
[`images/`](images/) folder.

## Printing information

- Material: ABS for all parts, except the front hull guard
  (`13_front_hull_guard.stl`), which should be printed in TPU.
- Layer height: 0.2 mm.
- General infill: 10%.
- Infill for `08_module_hull_handle.stl`: 30%.
- Infill for `13_front_hull_guard.stl`: 5%.

Before starting the assembly, note that a 0.2 mm clearance was left for the
part fits. Because of this, it may be necessary to sand some joints before the
final assembly.

## Parts to print

| File | Description | Quantity |
| --- | --- | ---: |
| `01_front_hull.stl` | Front hull | 2 |
| `02_right_middle_hull.stl` | Right middle hull | 1 |
| `03_left_middle_hull.stl` | Left middle hull | 1 |
| `04_right_rear_hull.stl` | Right rear hull | 1 |
| `05_left_rear_hull.stl` | Left rear hull | 1 |
| `06_small_hull_cover.stl` | Small cover for the front and rear hull sections | 4 |
| `07_large_hull_cover.stl` | Large cover for the middle hull section | 2 |
| `08_module_hull_handle.stl` | Handle that connects the hull to the central module | 4 |
| `09_motor_base.stl` | Motor base | 2 |
| `10_motor_base_cover.stl` | Motor base cover | 2 |
| `11_module.stl` | Central module | 1 |
| `12_module_cover.stl` | Central module cover | 1 |
| `13_front_hull_guard.stl` | Front hull guard | 2 |
| `14_front_platform_base.stl` | Front platform base | 2 |
| `15_rear_platform_base.stl` | Rear platform base | 2 |
| `16_large_platform_channel.stl` | Large platform channel | 4 |
| `17_small_platform_channel.stl` | Small platform channel | 4 |
| `18_large_platform_channel_support.stl` | Support for the large platform channels | 2 |
| `19_platform_base_diagonal_channel_support_1.stl` | Main diagonal platform base support | 2 |
| `20_platform_base_diagonal_channel_support_2.stl` | Secondary diagonal platform base support | 2 |
| `21_small_platform_channel_support.stl` | Support for the small platform channels | 2 |
| `22_platform_center_support.stl` | Platform center support | 1 |

## Additional items

| Item | Use | Quantity |
| --- | --- | ---: |
| M4 25 mm screw + nut + washer | Fastening the bases, handles, and platform | 24 units |
| M3 16 mm screw + nut + washer | Smaller fasteners | 2 units |
| Sealing silicone | Sealing the hull joints | As needed |
| Bicycle spokes | Auxiliary platform structure | 4 units |
| Hot glue | Locking and finishing the platform joints | As needed |
| Nylon line | Platform side protection | As needed |
| 50 x 30 cm acrylic sheet | Platform floor | 1 unit |

## Assembly steps

### 1. Hull assembly

1. Assemble the right hull by fitting parts `01`, `02`, and `04` together.
2. Assemble the left hull by fitting parts `01`, `03`, and `05` together.
3. Before making each final joint, apply sealing silicone to the connections to
   prevent water from entering.
4. Install the front hull guards `13` on the bow of both hulls.
5. Wait for the silicone to cure according to the manufacturer's
   recommendation before moving on to the next steps.

### 2. Connecting the hulls to the central module

1. With both hulls assembled, connect the four handles `08` to the hulls.
2. Connect the four handles `08` to the central module `11`.
3. Install the motor bases `09` and their covers `10`.
4. Install the large middle hull covers `07`.
5. Fasten the motor bases `09` and the handles `08` using M4 screws.

### 3. Assembly without the platform

1. Fit the four small hull covers `06` onto the front and rear hull sections.
2. Check that all covers are properly seated and sealed.

### 4. Assembly with the platform

1. Connect two large platform channels `16` using the support `18`.
2. Fit the 50 x 30 cm acrylic sheet.
3. Close the structure by adding the small platform channels `17` with the
   supports `21`.
4. Connect the diagonal supports `19` and `20`.
5. From the underside of the platform, connect the bicycle spokes to the center
   support `22` and to the channel supports `18` and `21`.
6. Thread the nylon line through the holes in the channels. This line acts as
   protection to prevent the aerial vehicle from leaving the platform.
7. Apply hot glue between the platform joints.
8. With the platform ready, connect the bases `14` and `15` to the boat hulls
   and to the platform.
9. Fasten the four corners with M4 screws.

### 5. Electronics

All electronics should be placed inside the central module `11`. The motor
wires should pass through the rear handles `08` and go up through the motor
bases `09`.
