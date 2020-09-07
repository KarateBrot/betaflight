/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Original implementation by Jan Post (2020).
 *
 * This file provides support for doubly linked lists in Betaflight.
 * Complexity for accessing data is O(1) at best and O(n) at worst.
 */


#pragma once


#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>


typedef struct linkedList_s linkedList_t;
typedef struct node_s node_t;


// - - - - -


typedef struct node_s {
	void *data;
	node_t *next;
	node_t *prev;
	linkedList_t *parent;
} node_t;

node_t *new_node(linkedList_t *list, void *data, size_t dataSize);
node_t *delete_node(node_t *node);

node_t *nodeIterator(node_t *node, int8_t steps);


// - - - - -


typedef struct linkedList_s {
	size_t dataSize;
	size_t size;
	size_t cursorIndex;
	node_t *cursorNode;
	node_t *head;
	node_t *tail;
} linkedList_t;

// Don't forget to delete_linkedList() before it runs out of scope!
linkedList_t *new_linkedList(size_t dataSize);
void delete_linkedList(linkedList_t *list);

// For pointer initialization use new_linkedList() instead
void linkedListInit(linkedList_t *list, size_t dataSize);

// Getter
node_t *linkedListFind(linkedList_t *list, size_t index);

// Adding list elements
//bool linkedListInsert(linkedList_t *list, void *data, size_t index, bool insertAfter);
bool linkedListPush(linkedList_t *list, void *data);
bool linkedListPushBack(linkedList_t *list, void *data);

// Deleting list elements
//node_t *linkedListExtract(linkedList_t *list, size_t index);
bool linkedListPop(linkedList_t *list);
bool linkedListPopBack(linkedList_t *list);
bool linkedListRemove(linkedList_t *list, node_t *node);
bool linkedListRemoveIndex(linkedList_t *list, size_t index);
bool linkedListClear(linkedList_t *list);

// Adding list elements (specific data structs)
bool linkedListPushInt8(linkedList_t *list, int8_t val);
bool linkedListPushBackInt8(linkedList_t *list, int8_t val);
bool linkedListPushInt16(linkedList_t *list, int16_t val);
bool linkedListPushBackInt16(linkedList_t *list, int16_t val);
bool linkedListPushFloat(linkedList_t *list, float val);
bool linkedListPushBackFloat(linkedList_t *list, float val);

// Getter (specific data structs)
int8_t linkedListGetInt8(linkedList_t *list, size_t index);
int16_t linkedListGetInt16(linkedList_t *list, size_t index);
float linkedListGetFloat(linkedList_t *list, size_t index);