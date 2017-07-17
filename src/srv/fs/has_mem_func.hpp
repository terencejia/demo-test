#pragma once

#include <type_traits>

template<typename, typename T>
struct has_mem_func {
	static_assert(
		std::integral_constant<T, false>::value,
		"Second template parameter needs to be of function type.");
};

template<typename C, typename Ret, typename... Args>
struct has_mem_func<C, Ret(Args...)> {
private:
	template<typename T>
	static constexpr auto check(T *)
	-> typename
	std::conditional<
		std::is_same<
			decltype( std::declval<T>().react( std::declval<Args>()... ) ),
			Ret
		>::value,
		std::true_type,
		std::false_type
	>::type;

	template<typename T>
	static constexpr std::false_type check(...);

	typedef decltype(check<C>(nullptr)) type;

public:
	static constexpr bool value = type::value;
};
