#ifndef SIGN_H
#define SIGN_H

namespace Sign {
	template <typename T> int Sign(T val) {
		return (T(0) < val) - (val < T(0));
	}
}

#endif // SIGN_H
