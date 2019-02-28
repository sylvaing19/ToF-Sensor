#ifndef _MEDIAN_h
#define _MEDIAN_h


template<typename T, size_t const BUFFER_SIZE>
class Median
{
public:
	Median()
	{
		reset();
	}

	void reset()
	{
		for (size_t i = 0; i < BUFFER_SIZE; i++)
		{
			rawBuffer[i] = (T)0;
			sortedBuffer[i] = (T)0;
		}
		insertionIndex = 0;
	}
	
	void add(T element)
	{
		rawBuffer[insertionIndex] = element;
		insertionIndex = (insertionIndex + 1) % BUFFER_SIZE;
		sortBuffer();
	}

	T value() const
	{
		return sortedBuffer[BUFFER_SIZE / 2];
	}

private:
	T rawBuffer[BUFFER_SIZE];
	T sortedBuffer[BUFFER_SIZE];
	size_t insertionIndex;

	void sortBuffer()
	{
		for (size_t i = 0; i < BUFFER_SIZE; i++)
		{
			sortedBuffer[i] = rawBuffer[i];
		}

		for (size_t j = 0; j < BUFFER_SIZE - 1; j++)
		{
			T min = sortedBuffer[j];
			size_t min_index = j;
			for (size_t i = j + 1; i < BUFFER_SIZE; i++)
			{
				if (sortedBuffer[i] < min)
				{
					min = sortedBuffer[i];
					min_index = i;
				}
			}
			swap(min_index, j);
		}
	}

	void swap(size_t i, size_t j)
	{
		T mem = sortedBuffer[i];
		sortedBuffer[i] = sortedBuffer[j];
		sortedBuffer[j] = mem;
	}
};


#endif
