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


#include "linkedlist.h"


node_t *new_node(linkedList_t *parent, void *data, size_t dataSize)
{
	node_t *newNode = (node_t*)malloc(sizeof(node_t));
	newNode->data = malloc(dataSize);
	for (size_t i = 0; i < dataSize; i++) {
		// Copy 8-bit chunks of data to newly allocated memory at node->data
		*((int8_t*)newNode->data + i) = *((int8_t*)data + i);
	}
	newNode->next = NULL;
	newNode->prev = NULL;
	newNode->parent = parent;
	return newNode;
}


// If deleted node is head or tail, a ptr to the new head or tail gets returned.
// The reason is to be able to recursively delete nodes from the front or back.
node_t *delete_node(node_t *node)
{
	if (node == NULL)
		return NULL;

	node_t *newEnd;
	// If node is unlinked (iteration terminating condition)
	if (node->prev == NULL && node->next == NULL) {
		newEnd = NULL;
	}
	// else if node is head (forward iteration)
	else if (node->prev == NULL) {
		newEnd = node->next;
		newEnd->prev = NULL;
	}
	// else if node is tail (backward iteration)
	else if (node->next == NULL) {
		newEnd = node->prev;
		newEnd->next = NULL;
	}
	// else if node is doubly linked (iteration terminating condition)
	else {
		newEnd = NULL;
	}
	free(node->data);
	free(node);
	return newEnd;
}


node_t *nodeIterator(node_t *node, int8_t steps)
{
	if (node == NULL || steps == 0)
		return node;

	if (steps > 0) {
		for (uint8_t i = 0; i < steps; i++) {
			if (node->next != NULL) {
				node = node->next;
			}
		}
	} else {
		for (uint8_t i = 0; i < abs(steps); i++) {
			if (node->prev != NULL) {
				node = node->prev;
			}
		}
	}

	return node;
}


// - - - - -


linkedList_t *new_linkedList(size_t dataSize)
{
	linkedList_t *list = (linkedList_t*)malloc(sizeof(linkedList_t));
	list->dataSize = dataSize;
	list->size = 0;
	list->cursorIndex = 0;
	list->cursorNode = NULL;
	list->head = NULL;
	list->tail = NULL;
	return list;
}


void delete_linkedList(linkedList_t *list)
{
	linkedListClear(list);
	free(list);
}


void linkedListInit(linkedList_t *list, size_t dataSize)
{
	bool cleared = linkedListClear(list);
	if (!cleared) return;
	list->dataSize = dataSize;
}


node_t *linkedListFind(linkedList_t *list, size_t index)
{
	if (list == NULL || list->head == NULL || list->size <= index)
		return NULL;

	int32_t dist = index - list->cursorIndex;
	int32_t distHead = index;
	int32_t distTail = list->size - index - 1;

	if (abs(dist) <= distHead && abs(dist) <= distTail) {
		list->cursorNode = nodeIterator(list->cursorNode, dist);
	}
	else if (distHead < distTail) {
		list->cursorNode = nodeIterator(list->head, distHead);
	}
	else {
		list->cursorNode = nodeIterator(list->tail, -distTail);
	}
	list->cursorIndex = index;

	return list->cursorNode;
}


//bool linkedListInsert(linkedList_t *list, void *data, size_t index, bool insertAfter)
//{
//	// TODO: Make it work
//}


bool linkedListPush(linkedList_t *list, void *data)
{
	if (list == NULL)
		return false;

	node_t *newHead = new_node(list, data, list->dataSize);
	if (list->head == NULL) {
		list->head = newHead;
		list->tail = newHead;
		list->cursorIndex = 0;
		list->cursorNode = newHead;
	}
	else {
		list->head->prev = newHead;
		newHead->next = list->head;
		list->head = newHead;
		list->cursorIndex++;
	}
	list->size++;
	return true;
}


bool linkedListPushBack(linkedList_t *list, void *data)
{
	if (list == NULL)
		return false;

	node_t *newTail = new_node(list, data, list->dataSize);
	if (list->tail == NULL) {
		list->tail = newTail;
		list->head = newTail;
		list->cursorIndex = 0;
		list->cursorNode = newTail;
	}
	else {
		list->tail->next = newTail;
		newTail->prev = list->tail;
		list->tail = newTail;
	}
	list->size++;
	return true;
}



bool linkedListPop(linkedList_t *list)
{
	if (list == NULL)
		return false;

	// Chop off current head and assign new head
	list->head = delete_node(list->head);
	list->size--;

	list->cursorIndex = 0;
	list->cursorNode = list->head;
	return true;
}


bool linkedListPopBack(linkedList_t *list)
{
	if (list == NULL)
		return false;

	// Chop off current tail and assign new tail
	list->tail = delete_node(list->tail);
	list->size--;

	list->cursorIndex = 0;
	list->cursorNode = list->head;
	return true;
}


bool linkedListRemove(linkedList_t *list, node_t *node)
{
	if (list == NULL || node == NULL || node->parent != list)
		return false;

	if (node->prev != NULL && node->next != NULL) {
		node->prev->next = node->next;
		node->next->prev = node->prev;
	}
	else if (node->prev != NULL) {
		list->tail = node->prev;
		list->tail->next = NULL;
	}
	else if (node->next != NULL) {
		list->head = node->next;
		list->head->prev = NULL;
	}
	else {
		list->head = NULL;
		list->tail = NULL;
	}
	delete_node(node);
	list->size--;

	list->cursorIndex = 0;
	list->cursorNode = list->head;
	return true;
}


bool linkedListRemoveIndex(linkedList_t *list, size_t index)
{
	node_t *node = linkedListFind(list, index);
	return linkedListRemove(list, node);
}


bool linkedListClear(linkedList_t *list)
{
	if (list == NULL)
		return false;

	// Recursively chop off head until there is no node left
	while (list->head != NULL) {
		list->head = delete_node(list->head);
	}
	list->tail = NULL;
	list->size = 0;

	list->cursorIndex = 0;
	list->cursorNode = NULL;
	return true;
}


bool linkedListPushInt8(linkedList_t *list, int8_t val)
{
	if (list->dataSize < sizeof(int8_t))
		return false;

	return linkedListPush(list, (void*)&val);
}


bool linkedListPushBackInt8(linkedList_t *list, int8_t val)
{
	if (list->dataSize < sizeof(int8_t))
		return false;

	return linkedListPushBack(list, (void*)&val);
}


bool linkedListPushInt16(linkedList_t *list, int16_t val)
{
	if (list->dataSize < sizeof(int16_t))
		return false;

	return linkedListPush(list, (void*)&val);
}


bool linkedListPushBackInt16(linkedList_t *list, int16_t val)
{
	if (list->dataSize < sizeof(int16_t))
		return false;

	return linkedListPushBack(list, (void*)&val);
}


bool linkedListPushFloat(linkedList_t *list, float val)
{
	if (list->dataSize < sizeof(float))
		return false;

	return linkedListPush(list, (void*)&val);
}


bool linkedListPushBackFloat(linkedList_t *list, float val)
{
	if (list->dataSize < sizeof(float))
		return false;

	return linkedListPushBack(list, (void*)&val);
}


int8_t linkedListGetInt8(linkedList_t *list, size_t index)
{
	return *(int8_t*)linkedListFind(list, index)->data;
}


int16_t linkedListGetInt16(linkedList_t *list, size_t index)
{
	return *(int16_t*)linkedListFind(list, index)->data;
}


float linkedListGetFloat(linkedList_t *list, size_t index)
{
	return *(float*)linkedListFind(list, index)->data;
}