#ifndef ARUCOLITE_H
#define ARUCOLITE_H

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

// if the user didn't define a database, just use the first 50 ArUco of the 4x4
// generic openCV database
#ifndef ARUCO_DB
#define ARUCO_DB	ARUCO_DB_4X4_1000
#endif

#include "database.h"

#include "vector.h"

// define debug colors
enum {
	ADP_BLACK		= 0,
	ADP_GREEN		= 1,
	ADP_RED			= 2,
	ADP_BLUE		= 3,
	ADP_YELLOW		= 4,
	ADP_EDGE_PT_COLOR	= 5,
	ADP_MARKER_COLOR	= 6,
	ADP_MAGENTA		= 7,
	ADP_GRAY		= 8,
};

static const int debug_colors[] = {
	[ADP_BLACK] =		0x000000,
	[ADP_GREEN] =		0x00ff00,
	[ADP_RED] =		0xff0000,
	[ADP_BLUE] =		0x0000ff,
	[ADP_YELLOW] =		0xffff00,
	[ADP_EDGE_PT_COLOR] =	0x0000ff,
	[ADP_MARKER_COLOR] =	0xffffff,
	[ADP_MAGENTA] =		0xff00ff,
	[ADP_GRAY] =		0x808080,
};


/* this table was computed with:

int reverse_byte(int v) {
	v = ((v & 0xF0) >> 4) | ((v << 4) & 0xF0);
	v = ((v & 0xCC) >> 2) | ((v << 2) & 0xCC);
	v = ((v & 0xAA) >> 1) | ((v << 1) & 0xAA);
	return v;
}
const static uint8_t short_table[] = {
	0x0F, 0x2F, 0x4F, 0x6F, 0xCF, 0x8F, 0xAF, 0x0E,
	0x2E, 0x4E, 0x6E, 0xCE, 0x8E, 0xAE
};
for (int i = 0; i < 14; i++) {
	edge_table[short_table[i] ^ 0xFF] = 1;
	edge_table[reverse_byte(short_table[i]) ^ 0xFF] = 2;
}
for (int i = 0; i < 256; i++)
	printf("%d,", edge_table[i]);
*/

static const uint8_t edge_table[256] = {
	0,0,0,0, 0,0,0,0, 0,2,2,2, 2,2,2,2,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,2,2,2, 2,2,2,2,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
	1,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
};

// structure to hold the information about one aruco
struct aruco_t {
	// corner point coordinates on the image (floating point, subpixel)
	pt2d_t pt[4];
	// integer id of this aruco on the database
	int aruco_idx;
};

template <int FRAME_WIDTH, int FRAME_HEIGHT, int MAX_ARUCO_COUNT = 16, bool DEBUG = false>
class ArucoLite {
public:
	// publish the template parameters as constants
	static constexpr int frame_width = FRAME_WIDTH;
	static constexpr int frame_height = FRAME_HEIGHT;
	static constexpr bool debug_mode = DEBUG;

	// the frame to be processed must be loaded to this array
	uint8_t frame[FRAME_HEIGHT][FRAME_WIDTH];

	// the result of the frame processing is stored here
	aruco_t result[MAX_ARUCO_COUNT];
	int arucos_found;

	// debug frame only occupies space if DEBUG is true
	uint8_t debug_frame[FRAME_HEIGHT * DEBUG][FRAME_WIDTH * DEBUG];


	// process the frame in "frame" and fill in the aruco information
	void process(void) {
		debug_clear_frame();
		compute_local_contrast();
		build_segments();
		process_finish();
	}

protected:
	// some compile time computed constants
	static constexpr int ARUCO_BORDER = 1;
	static constexpr int TOTAL_BITS = (ARUCO_BITS + ARUCO_BORDER * 2);
	static constexpr int DB_BYTES = (((ARUCO_BITS * ARUCO_BITS) + 7) / 8);

	static constexpr int USABLE_WIDTH = FRAME_WIDTH & 0xFFFFFFF8;
	static constexpr int USABLE_HEIGHT = FRAME_HEIGHT & 0xFFFFFFF8;

	static constexpr int FRAME_MARGIN_X = (FRAME_WIDTH - USABLE_WIDTH) / 2;
	static constexpr int FRAME_MARGIN_Y = (FRAME_HEIGHT - USABLE_HEIGHT) / 2;

	static constexpr int FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT;
	static constexpr int USABLE_SIZE = USABLE_WIDTH * USABLE_HEIGHT;

	// constants related to local contrast --------------------------------
	static constexpr int CELL = 8;
	static constexpr int GRID_X = USABLE_WIDTH / CELL;
	static constexpr int GRID_Y = USABLE_HEIGHT / CELL;
	static constexpr int DELTA = 5;

	// constants related to edge processing --------------------------------
	static constexpr int MAX_EDGE_PTS = USABLE_HEIGHT * 4;
	static constexpr int ANGLE_DELTA = 4;

	// constants related to segment processing -----------------------------

	// maximum number of arucos we can try to find in one frame. Note that
	// this includes early processing of black areas that don't end up
	// looking like arucos at all
	static constexpr int MAX_ARUCOS = USABLE_SIZE / 850;

	// maximum number of segments we can find in one frame. We use only 16
	// bit integer to reference a segment, so we can not have more than 64k
	static constexpr int MAX_SEGMENTS = (USABLE_SIZE / 50 < 65535) ? USABLE_SIZE / 50 : 65535;
	static constexpr int MAX_SEGS_PER_LINE = USABLE_WIDTH / 6;


	// segment processing data ---------------------------------------------

	struct segment_t {
		uint16_t y;
		uint16_t start;
		uint8_t length;	// HI-RES: we must support bigger lengths
		uint8_t aruco;
		int16_t next;	// HI-RES: we may need to support more than 64k segments
	};

	struct line_segments_t {
		uint16_t idx[MAX_SEGS_PER_LINE];
		uint16_t count;
	};

	// data space sharing: we compute lc_sum -> lc_grid -> segments, so we
	// don't need to have lc_sum and segments at the same time
	union {
		struct {
			uint32_t lc_sum[GRID_Y][GRID_X];
		};
		struct {
			segment_t segments[MAX_SEGMENTS];
			int segment_count, free_segment;

			int16_t arucos[MAX_ARUCOS], aruco_seg_count[MAX_ARUCOS];
			int aruco_count, free_aruco;

			line_segments_t previous_line, new_line;
		};
	};

	uint8_t lc_grid[GRID_Y][GRID_X];

	// another data sharing opportunity: we compute first/last from
	// segments, then "edge" -> "edge_angle", so we don't need first/last
	// while computing edge angles. We can not share data with segments, as
	// the segments data has information on all arucos on the image, whereas
	// this sequence is done per aruco
	union {
		struct {
			int16_t first[FRAME_HEIGHT], last[FRAME_HEIGHT];
			int y_start, y_end;
		};
		struct {
			uint8_t edge_angle[MAX_EDGE_PTS];
			uint16_t edge_bucket[32];
		};
	};
	int16_t edge[MAX_EDGE_PTS][2];
	uint16_t edge_count;


	// methods to compute local contrast

	int get_lc_sum(int y, int x)
	{
		if (y < 0)
			return 0;
		if (x < 0)
			return 0;
		return lc_sum[y][x];
	}

	void compute_local_contrast(void)
	{
		uint32_t total, gy, gx, ix, iy, x, y, avg;

		for (gy = 0; gy < GRID_Y; gy++) {
			x = FRAME_MARGIN_X;
			for (gx = 0; gx < GRID_X; gx++) {
				total = 0;
				y = gy * CELL + FRAME_MARGIN_Y;

				for (iy = 0; iy < CELL; iy++, y++) {
					for (ix = 0; ix < CELL; ix++, x++)
						total += frame[y][x];
					x -= CELL;
				}

				x += CELL;

				if (gy != 0)
					total += lc_sum[gy - 1][gx];
				if (gx != 0)
					total += lc_sum[gy][gx - 1];
				if (gx != 0 && gy != 0)
					total -= lc_sum[gy - 1][gx - 1];

				lc_sum[gy][gx] = total;
			}
		}


		for (y = 0; y < GRID_Y; y++) {
			gy = y;
			if (gy < DELTA)
				gy = DELTA;
			if (gy > GRID_Y - DELTA - 1)
				gy = GRID_Y - DELTA - 1;

			for (x = 0; x < GRID_X; x++) {
				gx = x;
				if (gx < DELTA)
					gx = DELTA;
				if (gx > GRID_X - DELTA - 1)
					gx = GRID_X - DELTA - 1;

				avg = (get_lc_sum(gy-5, gx-5) + get_lc_sum(gy+5, gx+5) -
					get_lc_sum(gy-5, gx+5) - get_lc_sum(gy+5, gx-5)) /
					((DELTA * 2 + 1) * (DELTA * 2 + 1) * (CELL*CELL));

				// a perfect average would be "( * 256) >> 8",
				// by using 240 we put the threshold slightly
				// below so that a uniform surface doesn't
				// appear as random noise
				avg = (avg * 240) >> 8;

				lc_grid[y][x] = avg;
			}
		}
	}


	int16_t alloc_segment(void)
	{
		int16_t ret;

		if (free_segment == -1) {
			if (segment_count >= MAX_SEGMENTS)
				return -1;
			ret = segment_count;
			segment_count++;
			debug("alloc segment: %d, new\n", ret);
		} else {
			ret = free_segment;
			free_segment = segments[ret].next;
			debug("alloc segment: %d, free %d\n", ret, free_segment);
		}
		return ret;
	}

	void dealloc_segment(int16_t idx)
	{
		debug("dealloc segment: %d, free %d\n", idx, free_segment);
		segments[idx].next = free_segment;
		free_segment = idx;
	}


	int16_t alloc_aruco(void)
	{
		int16_t ret;

		if (free_aruco == -1) {
			if (aruco_count >= MAX_ARUCOS)
				return -1;
			ret = aruco_count;
			aruco_count++;
			debug("alloc aruco: %d, new\n", ret);
		} else {
			ret = free_aruco;
			free_aruco = arucos[ret];
			debug("alloc aruco: %d, free %d\n", ret, free_aruco);
		}
		arucos[ret] = -1;
		aruco_seg_count[ret] = 0;
		return ret;
	}

	void dealloc_aruco(int16_t idx)
	{
		debug("dealloc aruco: %d, free %d\n", idx, free_aruco);

		int next, seg_idx = arucos[idx];
		while  (seg_idx != -1) {
			next = segments[seg_idx].next;
			dealloc_segment(seg_idx);
			seg_idx = next;
		}
		arucos[idx] = free_aruco;
		aruco_seg_count[idx] = -1;
		free_aruco = idx;
	}

	void aruco_add_segment(int aruco_idx, int new_seg_idx, segment_t *new_seg)
	{
		debug("add segment: aruco %d, seg %d\n", aruco_idx, new_seg_idx);
		new_seg->aruco = aruco_idx;
		new_seg->next = arucos[aruco_idx];
		arucos[aruco_idx] = new_seg_idx;
		aruco_seg_count[aruco_idx]++;
	}

	int merge_aruco(int aruco1, int aruco2)
	{
		int main, merge, seg_idx, last;

		debug("merge aruco: aruco1 %d (size %d), aruco2 %d (size %d)\n",
			aruco1, aruco_seg_count[aruco1],
			aruco2, aruco_seg_count[aruco2]);

		if (aruco_seg_count[aruco1] > aruco_seg_count[aruco2]) {
			main = aruco1;
			merge = aruco2;
		} else {
			main = aruco2;
			merge = aruco1;
		}

		if (aruco_seg_count[merge] == -1)
			return main;

		// change all aruco fields on the merged aruco to the main
		seg_idx = arucos[merge];
		last = -1;
		while (seg_idx != -1) {
			debug("  merging segment %d\n", seg_idx);
			segments[seg_idx].aruco = main;
			last = seg_idx;
			seg_idx = segments[seg_idx].next;
		}

		if (last != -1) {
			segments[last].next = arucos[main];
			arucos[main] = arucos[merge];
		}
		arucos[merge] = -1;

		aruco_seg_count[main] += aruco_seg_count[merge];

		dealloc_aruco(merge);

		return main;
	}


	void check_valid_aruco_and_drop(int idx)
	{
		// if it's already deleted, just return
		if (aruco_seg_count[idx] == -1)
			return;
		// if the whole "aruco" has less than 20 segments, it's not good
		// and needs to be dropped
		if (aruco_seg_count[idx] > 20)
			return;
		dealloc_aruco(idx);
	}


	void process_advance_line(void)
	{
		segment_t *prev_seg, *new_seg;
		int i, j;

		// check if there are segments on the previous line that ended
		// and drop their potential aruco's if they are clearly bogus
		for (i = 0; i < previous_line.count; i++) {
			prev_seg = &segments[previous_line.idx[i]];
			for (j = 0; j < new_line.count; j++) {
				new_seg = &segments[new_line.idx[j]];
				if (new_seg->aruco == prev_seg->aruco)
					break;
			}
			if (j == new_line.count)
				check_valid_aruco_and_drop(prev_seg->aruco);
		}

		// move the new line data to previous line
		memcpy(previous_line.idx, new_line.idx, sizeof(new_line.idx[0]) * new_line.count);
		previous_line.count = new_line.count;

		new_line.count = 0;
	}

	// return true if the segments intersect horizontally
	int intersect(segment_t *seg1, segment_t *seg2)
	{
		if (seg1->start >= seg2->start + seg2->length)
			return 0;
		if (seg2->start >= seg1->start + seg1->length)
			return 0;
		return 1;
	}

	void process_segment(int y, int x1, int x2)
	{
		segment_t *seg, *new_seg;
		int i, new_seg_idx, aruco_idx;

		// don't accept segments larger than 255 pixels: we would need
		// more than one byte to store them and they are unlikely to be
		// from a valid aruco as it would have to fill almost the entire
		// frame
		if (x2 - x1 > 255)
			return;

		mono_frame_draw_black_segment(y, x1, x2);

		new_seg_idx = alloc_segment();
		if (new_seg_idx == -1)
			return;

		new_seg = &segments[new_seg_idx];
		new_seg->y = y;
		new_seg->start = x1;
		new_seg->length = x2 - x1;

		if (new_line.count < MAX_SEGS_PER_LINE) {
			new_line.idx[new_line.count] = new_seg_idx;
			new_line.count++;
		}

		// check if this segment extends an aruco from a previous line
		aruco_idx = -1;
		for (i = 0; i < previous_line.count; i++) {
			seg = &segments[previous_line.idx[i]];
			if (intersect(seg, new_seg)) {
				if (aruco_idx == -1) {
					aruco_idx = seg->aruco;
				} else if (aruco_idx != seg->aruco) {
					aruco_idx = merge_aruco(aruco_idx, seg->aruco);
				}
			}
		}

		if (aruco_idx != -1) {
			aruco_add_segment(aruco_idx, new_seg_idx, new_seg);
		} else {
			aruco_idx = alloc_aruco();
			if (aruco_idx == -1) {
				dealloc_segment(new_seg_idx);
				return;
			}
			aruco_add_segment(aruco_idx, new_seg_idx, new_seg);
		}
	}

	void build_segments(void)
	{
		uint32_t x, y, ix, py, px, avg, edge;
		int segment_start;
		uint8_t *ptr, *frame_ptr;
		uint8_t shift, cell_shift = 0;

		segment_count = 0;
		free_segment = -1;
		aruco_count = 0;
		free_aruco = -1;

		previous_line.count = 0;
		new_line.count = 0;

		for (y = 0; y < USABLE_HEIGHT; y++) {
			py = y + FRAME_MARGIN_Y;
			ptr = lc_grid[y / CELL];
			segment_start = -1;
			shift = 0xAA;

			px = FRAME_MARGIN_X;
			frame_ptr = &frame[py][px];
			for (x = 0; x < GRID_X; x++) {
				avg = ptr[x];

				// loop unrolled for performance
				cell_shift = (*frame_ptr++ > avg);
				if (*frame_ptr++ > avg) cell_shift |= 0x02;
				if (*frame_ptr++ > avg) cell_shift |= 0x04;
				if (*frame_ptr++ > avg) cell_shift |= 0x08;
				if (*frame_ptr++ > avg) cell_shift |= 0x10;
				if (*frame_ptr++ > avg) cell_shift |= 0x20;
				if (*frame_ptr++ > avg) cell_shift |= 0x40;
				if (*frame_ptr++ > avg) cell_shift |= 0x80;

				if ((cell_shift == 0 || cell_shift == 0xFF) && (cell_shift & 15) == (shift & 15)) {
					px += CELL;
					continue;
				}

				for (ix = 0; ix < CELL; ix++, px++) {
					shift = (shift << 1) | (cell_shift & 1);
					cell_shift >>= 1;

					edge = edge_table[shift];
					if (edge == 0)
						continue;

					if (edge == 1) {
						//debug_plot(px - 4, py, GREEN);
						segment_start = px - 3;
					} else {
						//debug_plot(px - 3, py, RED);
						if (segment_start != -1) {
							process_segment(py, segment_start, px - 3);
							segment_start = -1;
						}
					}
				}
			}
			process_advance_line();
		}
		// do an extra process line to drop bad aruco's at the bottom of the frame
		process_advance_line();
	}




	int is_interior(int x, int y)
	{
		if (y <= y_start || y >= y_end)
			return 0;
		if (x == first[y] || x == last[y])
			return 0;
		return (x >= first[y-1] && x <= last[y-1] && x >= first[y+1] && x <= last[y+1]);
	}

	void add_edge(int x, int y)
	{
		if (edge_count >= MAX_EDGE_PTS)
			return;

		//debug_plot(x, y, EDGE_PT_COLOR);

		edge[edge_count][0] = x;
		edge[edge_count][1] = y;
		edge_count++;
	}

	void process_edge_point(int x, int y)
	{
		if (!is_interior(x, y))
			add_edge(x, y);
	}

	void process_edge_segment_fwd(int x1, int x2, int y)
	{
		for (int x = x1; x <= x2; x++)
			process_edge_point(x, y);
	}

	void process_edge_segment_rev(int x1, int x2, int y)
	{
		for (int x = x1; x >= x2; x--)
			process_edge_point(x, y);
	}

	void build_edge_points(void)
	{
		int y;

		edge_count = 0;

		process_edge_segment_fwd(first[y_start], last[y_start], y_start);

		for (y = y_start + 1; y < y_end; y++) {
			if (last[y-1] < last[y])
				process_edge_segment_fwd(last[y-1], last[y] - 1, y);
			process_edge_point(last[y], y);
			if (last[y+1] < last[y])
				process_edge_segment_rev(last[y] - 1, last[y+1], y);
		}

		process_edge_segment_rev(last[y_end], first[y_end], y_end);

		for (y = y_end - 1; y > y_start; y--) {
			if (first[y+1] > first[y])
				process_edge_segment_rev(first[y+1], first[y] + 1, y);
			process_edge_point(first[y], y);
			if (first[y-1] > first[y])
				process_edge_segment_fwd(first[y] + 1, first[y-1], y);
		}
	}

	int edge_pt(int idx)
	{
		if (idx < 0)
			return idx + edge_count;
		if (idx >= edge_count)
			return idx - edge_count;
		return idx;
	}

	// return a number between 0 and 255 that represents an "angle" between
	// [0 and 360[ degrees for the vector (x,y), but not exactly
	uint8_t approximate_atan2(int y, int x)
	{
		int t;

		if (x == 0 && y == 0)
			return 0;

		if (abs(y) > abs(x)) {
			t = (-x * 32) / y + 64;
			if (y < 0)
				t += 128;
		} else {
			t = (y * 32) / x;
			if (x < 0)
				t += 128;
		}
		return t;
	}


	// group edge points into angle buckets. The buckets overlap so that each point
	// goes into two buckets. This way there will always be a bucket where the edge
	// isn't split in half:
	//
	// 248 255 0   7 8  15 16 23 24 31 32 39 40 47
	//
	//  30    |     0     |     2     |     4     |  ...
	//
	//  |     31    |     1     |     3     |     5  ...

	int get_largest_bucket(void)
	{
		int i, max = 0, max_idx = 0;

		for (i = 0; i < 32; i++) {
			if (edge_bucket[i] > max) {
				max = edge_bucket[i];
				max_idx = i;
			}
		}
		return max_idx;
	}

	void comp_and_swap(int &a, int &b)
	{
		if (a < b)
			return;
		int t = a; a = b; b = t;
	}


	void rotate_corners(aruco_t *a, int rotation) {
		if (rotation == 0)
			return;
		pt2d_t tmp[4];
		memcpy(tmp, a->pt, sizeof(tmp));
		for (int e = 0; e < 4; e++)
			a->pt[e] = tmp[(e + 4 - rotation) & 3];
	}

	bool search_and_rotate(aruco_t *a, uint8_t *bmp)
	{
		for (int i = 0; i < ARUCO_DB_SIZE; i++) {
			for (int j = 0; j < 4; j++) {
				if (memcmp(bmp, database[i][j], DB_BYTES) == 0) {
					a->aruco_idx = i;
					rotate_corners(a, j);
					return true;
				}
			}
		}
		return false;
	}

	int mono_frame_pixel(uint32_t x, uint32_t y) {
		x -= FRAME_MARGIN_X;
		if (x >= USABLE_WIDTH)
			return 0;
		y -= FRAME_MARGIN_Y;
		if (y >= USABLE_HEIGHT)
			return 0;
		return frame[y + FRAME_MARGIN_Y][x + FRAME_MARGIN_X] > lc_grid[y / CELL][x / CELL];
	}

	// use the corner points to sample the aruco bits and identify it.
	// Rotate the corners so that pt[0] is always the top left corner of the
	// aruco and the other corners are sorted clockwise (pt[1] is top right,
	// etc.)
	bool identify_and_rotate(aruco_t *a) {
		pt2d_t vec[2], e[2], v, p;
		int i, j, ix, iy;
		int b, bit, b_idx, sample;
		uint8_t bmp[DB_BYTES];

		vec[0] = (a->pt[3] - a->pt[0]) * (1.0f / (TOTAL_BITS * 2));
		vec[1] = (a->pt[2] - a->pt[1]) * (1.0f / (TOTAL_BITS * 2));

		bit = 128;
		b_idx = 0;
		b = 0;
		for (i = 0; i < TOTAL_BITS; i++) {
			e[0] = a->pt[0] + vec[0] * (i * 2 + 1);
			e[1] = a->pt[1] + vec[1] * (i * 2 + 1);

			v = (e[1] - e[0]) * (1.0f / (TOTAL_BITS * 2));
			for (j = 0; j < TOTAL_BITS; j++) {
				p = e[0] + v * (j * 2 + 1);

				ix = p.x;
				if (ix < 0 || ix >= FRAME_WIDTH)
					return false;
				iy = p.y;
				if (iy < 0 || iy >= FRAME_HEIGHT)
					return false;

				sample = mono_frame_pixel(ix, iy);

				if (i < ARUCO_BORDER || i >= (TOTAL_BITS - ARUCO_BORDER) ||
				    j < ARUCO_BORDER || j >= (TOTAL_BITS - ARUCO_BORDER)) {
					if (sample != 0) {
						debug_plot(ix, iy, ADP_RED);
						return false;
					}
					debug_plot(ix, iy, ADP_MARKER_COLOR);
				} else {
					debug_plot(ix, iy, ADP_MARKER_COLOR);
					if (sample)
						b |= bit;
					bit >>= 1;
					if (bit == 0) {
						bit = 128;
						bmp[b_idx] = b;
						b_idx++;
						b = 0;
					}
				}
			}
		}

		const int reminder = ((ARUCO_BITS * ARUCO_BITS) & 7);
		if (reminder != 0) {
			b >>= (8 - reminder);
			bmp[b_idx] = b;
		}

		return search_and_rotate(a, bmp);
	}


	int compute_aruco_points(void)
	{
		int i, e, b, total, i1, i2, b0, b1, bucks[4];
		line_fit_t fit;
		line2d_t line[4];
		pt2d_t center;
		aruco_t a;

		memset(edge_bucket, 0, sizeof(edge_bucket));

		for (i = 0; i < edge_count; i++) {
			i1 = edge_pt(i - ANGLE_DELTA);
			i2 = edge_pt(i + ANGLE_DELTA);
			edge_angle[i] = approximate_atan2(edge[i2][1] - edge[i1][1], edge[i2][0] - edge[i1][0]);

			b0 = (edge_angle[i] / 16) * 2;
			b1 = (((edge_angle[i] + 8) / 16) * 2 + 31) & 31;
			edge_bucket[b0]++;
			edge_bucket[b1]++;

			//printf("%d %d %d %d %d\n", edge[i][0], edge[i][1], edge_angle[i], b0, b1);
		}

		// find the largest 4 buckets (ignoring neighbors)
		total = 0;
		for (i = 0; i < 4; i++) {
			bucks[i] = get_largest_bucket();
			total += edge_bucket[bucks[i]];
			edge_bucket[(bucks[i] + 31) & 31] = 0;
			edge_bucket[bucks[i]] = 0;
			edge_bucket[(bucks[i] + 1) & 31] = 0;
		}

		// ideally total should be the edge_count minus the corner points that
		// should be at most ANGLE_DELTA*2+1 per corner. In practice some points
		// from the corner area are still considered to be part of the edge, so
		// if we have less points than the minimum, it is probably not an aruco
		if (total < edge_count - (ANGLE_DELTA * 2 + 1) * 4)
			return 0;

		// sort the edges by angle, so that we get the edges in counter
		// clockwise mode
		comp_and_swap(bucks[0], bucks[2]);
		comp_and_swap(bucks[1], bucks[3]);
		comp_and_swap(bucks[0], bucks[1]);
		comp_and_swap(bucks[2], bucks[3]);
		comp_and_swap(bucks[1], bucks[2]);

		//printf("bucks %d %d %d %d\n", bucks[0], bucks[1], bucks[2], bucks[3]);

		// compute segments
		for (e = 0; e < 4; e++) {
			b = bucks[e];
			fit.reset();
			for (i = 0; i < edge_count; i++) {
				b0 = (edge_angle[i] / 16) * 2;
				b1 = (((edge_angle[i] + 8) / 16) * 2 + 31) & 31;
				if (b0 != b && b1 != b)
					continue;
				fit.add(edge[i][0] + 0.5, edge[i][1] + 0.5);
				debug_plot(edge[i][0], edge[i][1], ADP_EDGE_PT_COLOR);
			}
			// compute linear regression
			fit.compute(line[e]);

			//printf("segment %f %f %f %f\n", line[e].c.x, line[e].c.y, line[e].v.x, line[e].v.y);
			//draw_marker(line[e].x, line[e].y);
		}

		// we need to move the segments 0.5 pixels outwards, to compensate for
		// the fact that we are using the center of the pixels that are "inside"
		// the aruco

		// find an interior point (we select the center of the segment centers)
		center = pt2d_t(0, 0);
		for (e = 0; e < 4; e++)
			center += line[e].c;
		center *= 0.25;

		// make the "v" vector point in a consistent direction relative to the
		// center -> segment center vector and use it to move the center outwards
		for (e = 0; e < 4; e++) {
			if (line[e].v.cross(line[e].c - center) > 0)
				line[e].v = -line[e].v;
			// move the line outwards by half a pixel
			line[e].c += pt2d_t(line[e].v.y, -line[e].v.x) * 0.5;
		}

		// now we have all the segments we can intersect them
		for (e = 0; e < 4; e++)
			if (intersect_lines(line[e], line[(e + 1) & 3], a.pt[e]) == 0)
				return 0;

		if (!identify_and_rotate(&a))
			return 0;

		// printf("found aruco %d\n", a.aruco_idx);
		// for (e = 0; e < 4; e++)
		// 	printf("   corner %d: %g, %g\n", e, a.pt[e].x, a.pt[e].y);

		for (e = 0; e < 4; e++)
			debug_draw_marker(a.pt[e].x, a.pt[e].y, e + 1);

		// store result in result / arucos_found
		result[arucos_found] = a;
		arucos_found++;

		return 1;
	}


	int process_aruco(int idx)
	{
		segment_t *seg;
		int i, seg_idx, f, l, y;

		if (arucos_found >= MAX_ARUCO_COUNT)
			return 0;

		y_start = FRAME_HEIGHT;
		y_end = -1;

		memset(first, 0x10, sizeof(first));
		memset(last, 0xFF, sizeof(last));

		seg_idx = arucos[idx];
		while (seg_idx != -1) {
			seg = &segments[seg_idx];
			seg_idx = seg->next;

			f = seg->start;
			l = seg->start + seg->length - 1;
			y = seg->y;

			if (y < y_start) y_start = y;
			if (y > y_end) y_end = y;

			if (f < first[y]) first[y] = f;
			if (l > last[y]) last[y] = l;
		}

		// if the blob touches the border, we can't use it, or we'll risk having
		// one side of an aruco distorted by the frame border
		if (y_start <= FRAME_MARGIN_Y)
			return 0;
		if (y_end >= FRAME_HEIGHT - FRAME_MARGIN_Y - 1)
			return 0;

		// this is too small for an aruco
		if (y_end - y_start < 15) //PARAM
			return 0;

		// if there are sudden jumps at the border, it's not an aruco
		for (i = y_start + 5; i < y_end - 5; i++) {
			if (abs(first[i] - first[i+1]) > 50)	//PARAM
				return 0;
			if (abs(last[i] - last[i+1]) > 50)	//PARAM
				return 0;
		}

		// if it's too thin, it's not a good aruco
		f = 0;
		for (i = y_start; i <= y_end; i++) {
			l = last[i] - first[i];
			if (l > f) f = l;
		}
		if (f < 15) //PARAM
			return 0;

		// now that we passed all the fast criteria, try to fit a 4 side polygon
		// on the borders of the potential aruco
		build_edge_points();

		//printf("------------------ aruco %d --------------------\n", idx);
		return compute_aruco_points();
	}

	void process_finish(void)
	{
		arucos_found = 0;
		for (int i = 0; i < aruco_count; i++) {
			if (aruco_seg_count[i] != -1)
				process_aruco(i);
		}
	}


	// debug methods
	void debug_clear_frame(void) {
		if (!DEBUG)
			return;
		// gcc warns "‘memset’ used with length equal to number of
		// elements without multiplication by element size" without
		// checking that the size is actually zero, so we just disable
		// the warning for this memset. The zero case only happens in
		// not debug mode and the memset is not executed in that case
		#pragma GCC diagnostic push
		#pragma GCC diagnostic ignored "-Wmemset-elt-size"
			memset(debug_frame, ADP_GRAY, sizeof(debug_frame));
		#pragma GCC diagnostic pop
	}

	void debug_plot(int x, int y, int color) {
		if (!DEBUG)
			return;
		if (x < 0 || x >= FRAME_WIDTH)
			return;
		if (y < 0 || y >= FRAME_HEIGHT)
			return;
		debug_frame[y][x] = color;
	}

	void debug_draw_marker(float x, float y, int color) {
		if (!DEBUG)
			return;
		int i, ix, iy;

		ix = (int)x;
		iy = (int)y;
		for (i = -2; i <= 2; i++) {
			debug_plot(ix + i, iy, color);
			debug_plot(ix, iy + i, color);
		}
	}

	void mono_frame_draw_black_segment(int y, int x1, int x2) {
		if (!DEBUG)
			return;
		for (int x = x1; x < x2; x++)
			debug_plot(x, y, ADP_BLACK);
	}

	void debug(const char *fmt, ...) __attribute__((format (printf, 2, 3))) {
		//if (!DEBUG)
			return;
		va_list args;
		va_start(args, fmt);
		vprintf(fmt, args);
		va_end(args);
	}
};

#endif
