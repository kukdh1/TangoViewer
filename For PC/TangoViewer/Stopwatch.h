#pragma once

#ifndef _STOPWATCH_H_
#define _STOPWATCH_H_

#include <chrono>

namespace kukdh1
{
	class Stopwatch
	{
		private:
			std::chrono::system_clock::time_point start;
			std::chrono::system_clock::time_point end;

		public:
			void tic()
			{
				start = std::chrono::system_clock::now();
			}

			void tok()
			{
				end = std::chrono::system_clock::now();
			}

			double get()
			{
				return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
			}
	};
}

#endif