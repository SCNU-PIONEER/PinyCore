#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "PidBasic.hpp"

TEST_CASE("pid test", "[positional pid]")
{
    PositionalPid pid(1.0, 0.1, 0.01, 1, 10, 10);
    REQUIRE_THAT(pid.calc(0, 2), Catch::Matchers::WithinAbs(-2.11, 0.01));
    pid.reset();
    REQUIRE_THAT(pid.calc(2, 0), Catch::Matchers::WithinAbs(2.11, 0.01));
    pid.setParam(2.0, 0.2, 0.02, 1, 10, 10);
    pid.reset();
    REQUIRE_THAT(pid.calc(0, 2), Catch::Matchers::WithinAbs(-4.23, 0.01));
}

TEST_CASE("pid test", "[incremental pid]")
{
    IncrementalPid pid(1.0, 0.1, 0.01, 10);
    REQUIRE_THAT(pid.calc(0, 2), Catch::Matchers::WithinAbs(-2.22, 0.01));
    pid.reset();
    REQUIRE_THAT(pid.calc(2, 0), Catch::Matchers::WithinAbs(2.22, 0.01));
    pid.setParam(2.0, 0.2, 0.02, 10);
    pid.reset();
    REQUIRE_THAT(pid.calc(0, 2), Catch::Matchers::WithinAbs(-4.44, 0.01));
}
