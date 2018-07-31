#ifndef UTIL_H_
#define UTIL_H_

#include<memory>

namespace std{

template<typename T, typename... Ts>
unique_ptr<T> make_unique( Ts&&... params )
{
	return std::unique_ptr<T>( new T( std::forward<Ts>( params )... ) );
}

}

#endif

