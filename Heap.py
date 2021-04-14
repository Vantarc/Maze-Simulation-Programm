import numpy as np


class Heap:
    """Heap for the pathinding algorithm. The elements The lower the priority of an object is, the more it is at the front of the Heap.
    """
    def __init__(self, mapSize):
        self.items = np.zeros((mapSize[0]*mapSize[1], 3), dtype=int)
        self.number_of_items = 0

    def addTile(self, tileX, tileY, priority):
        self.items[self.number_of_items] = np.array((tileX, tileY, priority))
        self.sortUp(self.number_of_items)
        self.number_of_items += 1

    def updateTile(self, tileX, tileY, priority):
        """Updates the priority of the element with the given tile coords

        Args:
            tileX (int): XCoord of the tile
            tileY (int): YCoord of the tile
            priority (int): new priority of the tile
        """
        index = 0
        # find the current element in the queue
        for x in range(self.number_of_items):
            if self.items[x][0] == tileX and self.items[x][1] == tileY:
                index = x
        
        # save the old priority
        oldPriority = self.items[index,2]

        # update the priority
        self.items[index, 2] = priority

        # sort up/down based on the change of priority
        if oldPriority > priority:        
            self.sortUp(index)
        else:
            self.sortDown(index)

    def sortUp(self, index):
        """For a given index, this function sorts the elements based on the priority as far up in the heap as possible.

        Args:
            index (int): index of the element which should be sorted
        """
        # calculate parent
        parent_index = int(index - 1 / 2)
        
        # check if prio. of parent is smaller than prio. of  current element. If so: Swap the elements and try to sort the current element further 
        if self.compare(index, parent_index):
            self.swapItems(parent_index, index)
            self.sortUp(parent_index)

    def sortDown(self, index):
        """For a given index, this function sorts the elements based on the priority as far down in the heap as possible.

        Args:
            index (int): index of the element which should be sorted
        """
        child_index_left = index * 2 + 1
        child_index_right = index * 2 + 2


        # if left index doesn't exits(is bigger than number of items) item isn't at the bottom of the heap and can be sorted further
        if child_index_left >= self.number_of_items:
            return

        # indicates the index which will be swaped with the current index
        swap_index = child_index_left

        # check if priority of right index is bigger than priority of left index, so that the current element will always be swapped with the biggest child 
        if self.compare(child_index_right, child_index_left):
            swap_index = child_index_right

        # check if prio. of current element is smaller than prio. of most important child element. If so: Swap the elements and try to sort the current element further 
        if self.compare(swap_index, index):
            self.swapItems(swap_index, index)
            self.sortDown(swap_index)

    def reset(self):
        self.number_of_items = 0
    def getFirstTile(self):
        """Pops the first tile out of the heap.

        Returns:
            (int, int, int): the first element of the heap
        """
        firstTile = (self.items[0, 0], self.items[0, 1], self.items[0,2])

        self.number_of_items -= 1
        self.items[0] = self.items[self.number_of_items]

        self.sortDown(0)

        return firstTile

    def contains(self, tileX, tileY):
        """Check if the Heap already contains a tile

        Args:
            tileX (int): XCoord of the tile
            tileY (int): YCoord of the tile

        Returns:
            bool: True if tile is already in the heap. False if not
        """
        for x in range(self.number_of_items):
            if self.items[x][0] == tileX and self.items[x][1] == tileY:
                return True
        return False

    def isEmpty(self):
        """Checks if the Heap is empty

        Returns:
            bool: True if Heap is empty. False if Heap has elements
        """
        if self.number_of_items == 0:
            return True
        return False

    def compare(self, indexA, indexB):
        """Compares the importance of the elements at the specified indices

        Args:
            indexA (int): Index for the first element
            indexB (int): Index for the second element

        Returns:
            bool: True if A is more important. False if B is more important
        """
        # returns true if a is less important
        if self.items[indexA, 2] < self.items[indexB, 2]:
            return True
        return False

    def swapItems(self, indexA, indexB):
        """Swaps the elements at the specified indices

        Args:
            indexA (int): Index for the first element
            indexB (int): Index for the second element
        """
        valueA = (self.items[indexA, 0], self.items[indexA, 1], self.items[indexA, 2])
        self.items[indexA] = self.items[indexB]
        self.items[indexB] = np.array(valueA)