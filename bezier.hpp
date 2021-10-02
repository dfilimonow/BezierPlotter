#ifndef BEZIER_H
#define BEZIER_H

#include <iostream>
#include <functional>
#include <list>
#include <cmath>

namespace bezier
{

    namespace types
    {
        using real_t = double;
        using node_index_t = unsigned long;
        class point_2d
        {
        public:
            point_2d() = delete;
            point_2d(real_t, real_t);

            const real_t X, Y;

            bool operator==(const point_2d &) const;
            friend point_2d operator+(const point_2d &, const point_2d &);
            friend point_2d operator*(real_t, const point_2d &);
            friend point_2d operator*(const point_2d &, real_t);
            friend std::ostream &operator<<(std::ostream &, const point_2d &);     
        };

        point_2d::point_2d(real_t x, real_t y) : X(x), Y(y) {}

        std::ostream &operator<<(std::ostream &out, const point_2d &point)
        {
            out << "(" << point.X << ", " << point.Y << ")";
            return out;
        }

        point_2d operator+(const point_2d &lhs, const point_2d &rhs)
        {
            point_2d new_point(lhs.X + rhs.X, lhs.Y + rhs.Y);
            return new_point;
        }

        point_2d operator*(real_t scalar, const point_2d &point)
        {
            point_2d new_point(scalar * point.X, scalar * point.Y);
            return new_point;
        }

        point_2d operator*(const point_2d &point, real_t scalar)
        {
            point_2d new_point(scalar * point.X, scalar * point.Y);
            return new_point;
        }
    } // namespace types

    namespace constants
    {
        const int NUM_OF_CUBIC_BEZIER_NODES = 4;
        const types::real_t ARC = 4 * (sqrt(2) - 1) / 3;
    } // namespace constants
    template <typename T>
    T getNth(typename std::list<T>::const_iterator list, std::size_t size, types::node_index_t n){
        return 
            (n >= size) ? throw std::out_of_range("a curve node index is out of range"): 
            (n == 0) ? *list : getNth<T>(++list, size, n - 1);
    }

    using my_fun = std::function<const types::point_2d(types::node_index_t)>;

    std::function<std::function<types::point_2d(double)>(std::list<types::point_2d>, int, int)> Bez = [](std::list<types::point_2d> p, int deep, types::node_index_t pos) {
        return [p, deep, pos](double t) {
            return (deep != 1) ? ((1 - t) * Bez(p, deep - 1, pos)(t) + t * Bez(p, deep - 1, pos + 1)(t)) : (1 - t) * getNth<types::point_2d>(p.begin(), p.size(), pos) + t * getNth<types::point_2d>(p.begin(), p.size(), (pos + 1));
        };
    };

    std::function<my_fun(std::list<my_fun>)> curve =
        [](std::list<std::function<const types::point_2d(types::node_index_t)>> list) {
            return [list](types::node_index_t n) {
                return getNth<std::function<const types::point_2d(types::node_index_t)>>(list.begin(), list.size(), n / 4)(n % 4);
            };
        };


    std::function<std::function<const types::point_2d(types::node_index_t)>(std::list<std::function<const types::point_2d()>>)> segment =
        [](std::list<std::function<const types::point_2d()>> list) {
            return [list](types::node_index_t n) {
                return getNth<std::function<const types::point_2d()>>(list.begin(), list.size(), n)();
            };
        };

    std::function<std::function<const types::point_2d()>(types::real_t, types::real_t)> point = [](types::real_t x, types::real_t y) { return [x, y]() { return types::point_2d(x, y); }; };

    std::function<std::function<types::point_2d(types::node_index_t)>()> Cap = []() { 
        return curve(std::list{segment(std::list{point(-1, 1), point(-1, -1), point(1, -1), point(1, 1)})}); };

    std::function<std::function<types::point_2d(types::node_index_t)>()> Cup = []() { 
        return curve(std::list{segment(std::list{point(-1, -1), point(-1, 1), point(1, 1), point(1, -1)})});};

    std::function<std::function<types::point_2d(types::node_index_t)>()> ConvexArc = []() { 
        return curve(std::list{segment(std::list{point(0, 1), point(constants::ARC, 1), point(1, constants::ARC), point(1, 0)})});};

    std::function<std::function<types::point_2d(types::node_index_t)>()> ConcaveArc = []() { 
        return curve(std::list{segment(std::list{point(0, 1), point(0, 1 - constants::ARC), point(1 - constants::ARC, 0), point(1, 0)})});};

    std::function<std::function<types::point_2d(types::node_index_t)>(types::point_2d, types::point_2d)> LineSegment = 
        [](types::point_2d p, types::point_2d q) { 
        return curve(std::list{segment(std::list{point(p.X, p.Y), point(p.X, p.Y), point(q.X, q.Y), point(q.X, q.Y)})});};

    std::function<bool(std::function<const types::point_2d(types::node_index_t)>, types::node_index_t)> isViableSegment =
        [](std::function<const types::point_2d(types::node_index_t)> fun, types::node_index_t n) {
            try {
                return (fun(n).X  || true);
            }
            catch (const std::out_of_range &oor) {
                return false;
            }
        };

    // Move

     std::function<std::function<const types::point_2d(types::node_index_t)>(std::function<types::point_2d(types::node_index_t)>, types::node_index_t, types::real_t, types::real_t, types::node_index_t)> changeOneInSegment =
        [](std::function<types::point_2d(types::node_index_t)> fun, types::node_index_t n, types::real_t x, types::real_t y, types::node_index_t want) {
            return segment(std::list{(want == n) ? point(fun(n).X + x, fun(n).Y + y) : point(fun(n).X, fun(n ).Y),
                                     (want == n + 1) ? point(fun(n + 1).X + x, fun(n + 1).Y + y) : point(fun(n + 1).X, fun(n + 1).Y),
                                     (want == n + 2) ? point(fun(n + 2).X + x, fun(n + 2).Y + y) : point(fun(n + 2).X, fun(n + 2).Y),
                                     (want == n + 3) ? point(fun(n + 3).X + x, fun(n + 3).Y + y) : point(fun(n + 3).X, fun(n + 3).Y)});
        };

    std::function<std::list<my_fun>(std::list<my_fun> &, my_fun, types::node_index_t, types::real_t, types::real_t, types::node_index_t)>
        getCurveMove =
            [](std::list<my_fun> &list, my_fun fun, types::node_index_t n, types::real_t x, types::real_t y, types::node_index_t want) {
                (isViableSegment(fun, n) == 1) ? list.push_back(changeOneInSegment(fun, n, x, y, want)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveMove(list, fun, n + 4, x, y, want) : 
                       (want < n) ? list : throw std::out_of_range("a curve node index is out of range");
            };

    std::function<my_fun(my_fun, types::node_index_t, types::real_t, types::real_t)> MovePoint =
        [](my_fun fun, types::node_index_t want, types::real_t x, types::real_t y) {
            auto empty = std::list<my_fun>();
            return curve(getCurveMove(empty, fun, 0, x, y, want));
        };

    // Translate

    std::function<std::function<const types::point_2d(types::node_index_t)>(std::function<types::point_2d(types::node_index_t)>, types::node_index_t, types::real_t, types::real_t)> changeInSegment =
        [](std::function<types::point_2d(types::node_index_t)> fun, types::node_index_t n, types::real_t x, types::real_t y) {
            return segment(std::list{point(fun(n).X + x, fun(n).Y + y), point(fun(n + 1).X + x, fun(n + 1).Y + y),
                                     point(fun(n + 2).X + x, fun(n + 2).Y + y), point(fun(n + 3).X + x, fun(n + 3).Y + y)});
        };

    std::function<std::list<my_fun>(std::list<my_fun> &, my_fun, types::node_index_t, types::real_t, types::real_t)>
        getCurveTranslate =
            [](std::list<my_fun> &list, my_fun fun, types::node_index_t n, types::real_t x, types::real_t y) {
                (isViableSegment(fun, n) == 1) ? list.push_back(changeInSegment(fun, n, x, y)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveTranslate(list, fun, n + 4, x, y) : list;
            };

    std::function<my_fun(my_fun, types::real_t, types::real_t)> Translate =
        [](my_fun fun, types::real_t x, types::real_t y) {
            auto empty = std::list<my_fun>();
            return curve(getCurveTranslate(empty, fun, 0, x, y));
        };

     // Scale

    std::function<std::function<const types::point_2d(types::node_index_t)>(std::function<types::point_2d(types::node_index_t)>, types::node_index_t, types::real_t, types::real_t)> scaleSegment =
        [](std::function<types::point_2d(types::node_index_t)> fun, types::node_index_t n, types::real_t x, types::real_t y) {
             return segment(std::list{point(fun(n).X * x, fun(n).Y * y), point(fun(n + 1).X * x, fun(n + 1).Y * y),
                                      point(fun(n + 2).X * x, fun(n + 2).Y * y), point(fun(n + 3).X * x, fun(n + 3).Y * y)});
        };

    std::function<std::list<my_fun>(std::list<my_fun> &, my_fun, types::node_index_t, types::real_t, types::real_t)>
        getCurveScale =
            [](std::list<my_fun> &list, my_fun fun, types::node_index_t n, types::real_t x, types::real_t y) {
                (isViableSegment(fun, n) == 1) ? list.push_back(scaleSegment(fun, n, x, y)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveScale(list, fun, n + 4, x, y) : list;
            };

    std::function<my_fun(my_fun, types::real_t, types::real_t)> Scale =
        [](my_fun fun, types::real_t x, types::real_t y) {
            auto empty = std::list<my_fun>();
            return curve(getCurveScale(empty, fun, 0, x, y));
        };

    // Rotate

    std::function<std::function<const types::point_2d(types::node_index_t)>(std::function<types::point_2d(types::node_index_t)>, types::node_index_t, unsigned long)> rotateSegment =
        [](std::function<types::point_2d(types::node_index_t)> fun, types::node_index_t n, unsigned long a) {
            return segment(std::list{point(fun(n).X * cos(a * M_PI / 180) + (-1 * fun(n).Y * sin(a * M_PI / 180)), fun(n).Y * cos(a * M_PI / 180) + fun(n).X * sin(a * M_PI / 180)),
                                     point(fun(n + 1).X * cos(a * M_PI / 180) + (-1 * fun(n + 1).Y * sin(a * M_PI / 180)), fun(n + 1).Y * cos(a * M_PI / 180) + fun(n + 1).X * sin(a * M_PI / 180)),
                                     point(fun(n + 2).X * cos(a * M_PI / 180) + (-1 * fun(n + 2).Y * sin(a * M_PI / 180)), fun(n + 2).Y * cos(a * M_PI / 180) + fun(n + 2).X * sin(a * M_PI / 180)),
                                     point(fun(n + 3).X * cos(a * M_PI / 180) + (-1 * fun(n + 3).Y * sin(a * M_PI / 180)) , fun(n + 3).Y * cos(a * M_PI / 180) + fun(n + 3).X * sin(a * M_PI / 180))});
        };

    std::function<std::list<my_fun>(std::list<my_fun> &, my_fun, types::node_index_t, unsigned long a)>
        getCurveRotate =
            [](std::list<my_fun> &list, my_fun fun, types::node_index_t n, unsigned long a) {
                (isViableSegment(fun, n) == 1) ? list.push_back(rotateSegment(fun, n, a)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveRotate(list, fun, n + 4, a) : list;
            };

    std::function<my_fun(my_fun, unsigned long)> Rotate =
        [](my_fun fun, unsigned long a) {
            auto empty = std::list<my_fun>();
            return curve(getCurveRotate(empty, fun, 0, a));
        };

     // Concatenate

    std::function<std::function<const types::point_2d(types::node_index_t)>(std::function<types::point_2d(types::node_index_t)>, types::node_index_t)> concatSegment =
        [](std::function<types::point_2d(types::node_index_t)> fun, types::node_index_t n) {
            return segment(std::list{point(fun(n).X , fun(n).Y),
                                     point(fun(n + 1).X , fun(n + 1).Y),
                                     point(fun(n + 2).X , fun(n + 2).Y),
                                     point(fun(n + 3).X , fun(n + 3).Y)});
        };

    std::function<std::list<my_fun>(std::list<my_fun> &, my_fun, types::node_index_t, unsigned long a)>
        getCurveRotate =
            [](std::list<my_fun> &list, my_fun fun, types::node_index_t n, unsigned long a) {
                (isViableSegment(fun, n) == 1) ? list.push_back(rotateSegment(fun, n, a)) : (void)NULL;
                return (isViableSegment(fun, n) == 1) ? getCurveRotate(list, fun, n + 4, a) : list;
            };

    std::function<my_fun(std::initializer_list<my_fun>)> Concatenate =
        [](std::initializer_list<my_fun> funs) {
            auto empty = std::list<my_fun>();
            return curve(getCurveRotate(empty, fun, 0, a));
        };

    

} // namespace bezier

#endif // BENZIER_H
