# ArucoLite

ArucoLite is a library that processes an image to find ArUco barcodes in it and extract their corner positions. Unlike libraries like OpenCV, ArucoLite tries to use as little memory as possible.

To process a 324x324 image, it uses around 25kB of memory. As this was designed to run on micro-controllers, it doesn't use any dynamic memory allocation and all the memory used is part of the ArucoLite object. The class is designed as a template so that all the structure sizes and code can be optimized at compile time for the required frame size.

The ArucoLite class defines a "Frame" type that can be used to create a frame buffer to store an image to be processed by the library. By using this type, user code can define more than one frame object and use it to do double buffering, by capturing to one object using DMA while the library processes the image in the other object. This is an incompatible change with version 1, but it was necessary to allow more efficient buffering modes.

The library was also optimized for performance, and processes a 324x324 image on a RP2040 at 250MHz in about 22ms (exact timing depends on the image contents).

You may have noticed that the 324x324 number seems oddly peculiar. That's because it's the resolution of the HM01B0 camera for which there are breakout boards available and some Pi Pico compatible boards that already include the camera sensor.

The library was only tested with small resolutions but in theory it should adapt to the resolution requested.


## Usage

Before including the header file, you need to define ARUCO_DB to select the aruco database to use. It needs to be one of:

- ARUCO_DB_ORIGINAL
- ARUCO_DB_4X4_1000
- ARUCO_DB_5X5_1000
- ARUCO_DB_6X6_1000
- ARUCO_DB_7X7_1000
- ARUCO_DB_ARUCO_MIP_36h12
- ARUCO_DB_APRILTAG_16h5
- ARUCO_DB_APRILTAG_25h9
- ARUCO_DB_APRILTAG_36h10
- ARUCO_DB_APRILTAG_36h11

If the database is not selected, the default is ARUCO_DB_1000_4x4. You can also define ARUCO_DB_SIZE to restrict the database to the first N entries in the database. If the size is not specified it defaults to the total size of the database selected.

Limiting the size of the database saves flash space, speeds up the search for a matching aruco and avoids spurious matches with arucos that you are actually not using.

After including the ArucoLite.h header file, you must declare an ArucoLite object. The class is a template for efficiency, that takes 4 parameters:

**int width**, **int height**: dimension of the image to be processed
**int max_arucos**: maximum number of arucos that can be detected on a single frame
**bool debug**: run in debug mode, producing a debug frame that shows the features detected on the image (edges, corners, bit positions, etc.). This uses more memory (an extra frame) and more processing time

A small example code looks like this:

```cpp
//#define ARUCO_DB ARUCO_DB_ORIGINAL
#define ARUCO_DB ARUCO_DB_4X4_1000
//#define ARUCO_DB ARUCO_DB_5X5_1000
//#define ARUCO_DB ARUCO_DB_6X6_1000
//#define ARUCO_DB ARUCO_DB_7X7_1000
//#define ARUCO_DB ARUCO_DB_ARUCO_MIP_36h12
//#define ARUCO_DB ARUCO_DB_APRILTAG_16h5
//#define ARUCO_DB ARUCO_DB_APRILTAG_25h9
//#define ARUCO_DB ARUCO_DB_APRILTAG_36h10
//#define ARUCO_DB ARUCO_DB_APRILTAG_36h11

#define ARUCO_DB_SIZE 100

// include the header file after selecting the aruco database to use
#include <ArucoLite.h>

// declare the type to be used with the resolution and parameters we want
typedef ArucoLite<324, 324, 16, false> MyAruco_t;

// declare an ArucoLite object as a global object. Don't declare
// it on the stack as it will likely use too much stack
static MyAruco_t Aruco;

// create one or more frame objects to store the frame captured by
// the camera
static MyAruco_t::Frame frame;


setup() {
        // nothing to do here, the object doesn't require initialization
}

loop() {
        // capture image from the camera into the "frame" object
        camera_capture(frame);

        // call process to find aruco
        Aruco.process(frame);

        // check the results and print the data of the aruco's found
        printf("found %d arucos\n", Aruco.arucos_found);
        for (int i = 0; i < Aruco.arucos_found; i++) {
                aruco_t &aruco = Aruco.result[i];
                printf("aruco %d: ", i);
                for (int p = 0; p < 4; p++)
                        printf("(%g, %g), ", aruco.pt[p].x, aruco.pt[p].y);
                printf("id %d\n", aruco.aruco_idx);
        }
}
```

For a more complete example, check the example code that comes with the library.

After calling process(), the number of arucos found is stored in ```arucos_found``` and the detail of each aruco is stored in the ```result``` array. Each entry in the array has the id of the aruco (its index in the database) and the X/Y position of each of the 4 corners of the aruco. The library rotates the aruco appropriately so that, no matter what the position of the aruco is in the image, the 4 corners identified are always the same on the barcode.

The X/Y coordinates of the corners are floating point numbers, because the library tries to determine the corner positions with sub-pixel resolution. The top left of the image is coord (0,0) and bottom right is (width,height). The middle of the top left pixel is (0.5,0.5).


## CODE32 (Experimental)

CODE32 aruco is an experimental feature that uses a 32 bit mask to write the interior bits of a 6x6 aruco. The idea is to allow the user to explore different formats, error correction methods, etc., without having to produce a database and/or change the library in any way.

The inner corners of the aruco are fixed, with the top left being white and the others all black. This way the library can detect rotation and process it like it does for all other aruco formats.

The aruco is built like this:

|  |  |  |  |  |  |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **white** | b31 | b30 | b29 | b28 | **black** |
| b27 | b26 | b25 | b24 | b23 | b22 |
| b21 | b20 | b19 | b18 | b17 | b16 |
| b15 | b14 | b13 | b12 | b11 | b10 |
| b09 | b08 | b07 | b06 | b05 | b04 |
| **black** | b03 | b02 | b01 | b00 | **black** |

where b31 is the most significant bit of the 32 bit mask and b0 is the least significant bit.

The CODE32 arucos can be generated using the aruco-database-converter utility (also available on github) that can generate an SVG file with multiple arucos on the same document. When the library recognizes a CODE32 aruco it returns the same 32 bit number that was passed to the aruco-database-converter utility to generate the image.

Note that a 6x6 aruco database is computed to space the arucos evenly in binary space to maximize the hamming distance between any two arucos. This means that a misread bit (or a few bits) in one aruco should not allow it to be confused with another aruco on the same database.

CODE32 applies a pseudo-random 32bit bijective transformation to make it more likely that a code that is misread in the image produces an invalid code ofr the application.

For instance, let's say the application produces two arucos with code 2 and 3. If encoded directly, a single bit error could misread one as the other. However, code 2 is encoded as 0xfa59460e and code 3 as 0x21a87429. A single bit error is very unlikely to produce a valid code.

In theory the robustness of this method depends only on the number of valid codes. Since there are 2^32 possible codes, the probability of a misread code returning one of N valid codes is N / 2^32. For instance, if the application has 4295 valid codes, there is a one in a million chance of misreading a code.
