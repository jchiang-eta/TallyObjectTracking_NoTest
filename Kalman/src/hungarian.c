//
//(Minimum) Assignment Problem by Hungarian Algorithm
//taken from Knuth's Stanford Graphbase
//
#include <stdio.h>
#include <stdint.h>

#include "hungarian.h"
#include "pipeline_config.h"
#include "debug.h"
#include "assertion.h"

#define INF (0x7FFFFFFF)

#define	max(a, b)	((a) >= (b) ? (a) : (b))
#define	min(a, b)	((a) <= (b) ? (a) : (b))

#define DEBUG

uint8_t hungarian(float* cost_matrix, int n_rows, int n_cols, vec2ui8* matchings, int* costs, hungarian_t* hun_obj)
{

	debug("Hungarian\n");

	int8_t* col_mate = hun_obj->col_mate;
	int8_t* row_mate = hun_obj->row_mate;
	int8_t* parent_row = hun_obj->parent_row;
	int8_t* unchosen_row = hun_obj->unchosen_row;
	int* row_dec = hun_obj->row_dec;
	int* col_inc = hun_obj->col_inc;
	int* slack = hun_obj->slack;
	int8_t* slack_row = hun_obj->slack_row;
	uint8_t _h_rows = hun_obj->_h_rows;
	uint8_t _h_cols = hun_obj->_h_cols;
	int* hun_costmat = hun_obj->hun_costmat;

	assert(n_rows <= _h_rows);
	assert(n_cols <= _h_cols);

	// in case we are overflowing array, return immediately
	if (n_rows > _h_rows || n_cols > _h_cols){
		debug("Matrix overflow - stopping execution\n");
		return 0;
	}

	int i, j;
	const int false = 0, true = 1;
	//int m=MAX_SIZE_ROW, 
	//int n=MAX_SIZE_COL;
	int k;
	int l;
	int s;

	int t;
	int q;

	int unmatched;
	int cost = 0;
	int count;

	uint8_t n_matchings;

	// initialize all costs to very high
	for (i = 0; i < _h_rows; ++i) {
		for (j = 0; j < _h_cols; ++j) {
			hun_costmat[i * _h_cols + j] = INF/2; // half the slack
		}
	}

	// copy our costs into the subsection of cost array
	for (i = 0; i < n_rows; i++) {
		for (j = 0; j < n_cols; j++)
		{
			hun_costmat[i * _h_cols + j] = (int)cost_matrix[i * n_cols + j];
		}
	}

int max_matches = MIN(n_rows, n_cols);

#ifdef DEBUG
	debug("\r\n --- Input data ---\r\n");
	debug("\r\n Rows: %d, Cols: %d\r\n", n_rows, n_cols);
	for (i = 0; i < n_rows; i++) {
		for (j = 0; j < n_cols; j++)
		{
			debug("%d ", (int)cost_matrix[i * n_cols + j]);
		}
		debug("\r\n");
	}
	debug("\r\n");

	debug("\r\n --- Cost matrix: ---\r\n");
	debug("\r\n Rows: %d, Cols: %d\r\n", _h_rows, _h_cols);
	for (i = 0; i < _h_rows; ++i) {
		for (j = 0; j < _h_cols; ++j)
			debug("%d ", hun_costmat[i * _h_cols + j]);
		debug("\r\n");
	}
#endif

	// Begin subtract column minima in order to start with lots of zeroes
	for (l = 0; l < _h_cols; l++)
	{
		s = hun_costmat[l];
		for (k = 1; k < _h_cols; k++)
			if (hun_costmat[k * _h_cols + l] < s)
				s = hun_costmat[k * _h_cols + l];
		cost += s;
		if (s != 0)
			for (k = 0; k < _h_cols; k++)
				hun_costmat[k * _h_cols + l] -= s;
	}
	// End subtract column minima in order to start with lots of zeroes

	// Begin initial state
	t = 0;
	for (l = 0; l < _h_cols; l++)
	{
		row_mate[l] = -1;
		parent_row[l] = -1;
		col_inc[l] = 0;
		slack[l] = INF;
	}

	for (k = 0; k < _h_rows; k++)
	{
		s = hun_costmat[k * _h_cols];
		for (l = 1; l < _h_cols; l++)
			if (hun_costmat[k * _h_cols + l] < s)
				s = hun_costmat[k * _h_cols + l];
		row_dec[k] = s;
		for (l = 0; l < _h_cols; l++)
			if (s == hun_costmat[k * _h_cols + l] && row_mate[l] < 0)
			{
				col_mate[k] = l;
				row_mate[l] = k;
#ifdef DEBUG
				debug("matching col %d==row %d\n", l, k);
#endif
				goto row_done;
			}
		col_mate[k] = -1;
#ifdef DEBUG
		debug("node %d: unmatched row %d\n", t, k);
#endif
		unchosen_row[t++] = k;
	row_done:
		;
	}
	// End initial state 16

	// Begin Hungarian algorithm 18
	if (t == 0)
		goto done;
	unmatched = t;
	while (1)
	{
#ifdef DEBUG
		debug("Matched %d rows.\n", _h_rows - t);
#endif
		q = 0;
		while (1)
		{
			while (q < t) {
				// Begin explore node q of the forest 19
				k = unchosen_row[q];
				s = row_dec[k];
				for (l = 0; l < _h_cols; l++) {
					if (slack[l]) {
						int del;
						del = hun_costmat[k * _h_cols + l] - s + col_inc[l];
						if (del < slack[l]) {
							if (del == 0) {
								if (row_mate[l] < 0)
									goto breakthru;
								slack[l] = 0;
								parent_row[l] = k;
#ifdef DEBUG
								debug("node %d: row %d==col %d--row %d\n",
									t, row_mate[l], l, k);
#endif
								unchosen_row[t++] = row_mate[l];
							}
							else {
								slack[l] = del;
								slack_row[l] = k;
							}
						}
					}
				}
				// End explore node q of the forest 19
				q++;
			}

			// Begin introduce a new zero into the matrix 21
			s = INF;
			for (l = 0; l < _h_cols; l++)
				if (slack[l] && slack[l] < s)
					s = slack[l];
			for (q = 0; q < t; q++)
				row_dec[unchosen_row[q]] += s;
			for (l = 0; l < _h_cols; l++)
				if (slack[l]) {
					slack[l] -= s;
					if (slack[l] == 0) {
						// Begin look at a new zero 22
						k = slack_row[l];
#ifdef DEBUG
						debug("Decreasing uncovered elements by %d produces zero at [%d,%d]\n", s, k, l);
#endif
						if (row_mate[l] < 0) {
							for (j = l + 1; j < _h_cols; j++)
								if (slack[j] == 0)
									col_inc[j] += s;
							goto breakthru;
						}
						else {
							parent_row[l] = k;
#ifdef DEBUG
							debug("node %d: row %d==col %d--row %d\n", t, row_mate[l], l, k);
#endif
							unchosen_row[t++] = row_mate[l];
						}
						// End look at a new zero 22
					}
				}
				else
					col_inc[l] += s;
			// End introduce a new zero into the matrix 21
		}
	breakthru:
		// Begin update the matching 20
#ifdef DEBUG
		debug("Breakthrough at node %d of %d!\n", q, t);
#endif
		int while_cnt = 100;
		while (while_cnt--) {
			j = col_mate[k];
			col_mate[k] = l;
			row_mate[l] = k;
#ifdef DEBUG
			debug("rematching col %d==row %d\n", l, k);
#endif
			if (j < 0)
				break;
			k = parent_row[j];
			l = j;
		}
		debug("while_cnt = %d\n", while_cnt);
		if (while_cnt == -1) {
			return 0;
		}
		// End update the matching 20
		if (--unmatched == 0)
			goto done;
		// Begin get ready for another stage 17
		t = 0;
		for (l = 0; l < _h_cols; l++) {
			parent_row[l] = -1;
			slack[l] = INF;
		}
		for (k = 0; k < _h_rows; k++)
			if (col_mate[k] < 0) {
#ifdef DEBUG
				debug("node %d: unmatched row %d\n", t, k);
#endif
				unchosen_row[t++] = k;
			}
		// End get ready for another stage 17
	}
done:

	n_matchings = max(n_rows, n_cols);
#ifdef DEBUG
	debug("catndidate matches: %d\r\n", n_matchings);
#endif

	count = 0;
	for (i = 0; i < n_matchings; ++i) {
#ifdef DEBUG
		debug("[%d,%d]\r\n", i, col_mate[i]);
#endif
		if (col_mate[i] < n_cols && i < n_rows) {
			matchings[count].v[0] = i;
			matchings[count].v[1] = col_mate[i];
			costs[count] = (int)cost_matrix[i * n_cols + col_mate[i]];
			count++;
		}
	}

	n_matchings = min(max_matches, count);

#ifdef DEBUG
	debug("final matches: %d\r\n", n_matchings);
#endif

	return  n_matchings;
}