/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <gtest/gtest.h>
#include <nav_core2/exceptions.h>
#include <string>

TEST(Exceptions, direct_code_access)
{
  // Make sure the caught exceptions have the same types as the thrown exception
  // (This version does not create any copies of the exception)
  try
  {
    throw nav_core2::GlobalPlannerTimeoutException("test case");
  }
  catch (nav_core2::GlobalPlannerTimeoutException& x)
  {
    EXPECT_EQ(x.getResultCode(), 12);
  }

  try
  {
    throw nav_core2::GlobalPlannerTimeoutException("test case");
  }
  catch (nav_core2::PlannerException& x)
  {
    EXPECT_EQ(x.getResultCode(), 12);
  }
}

TEST(Exceptions, indirect_code_access)
{
  // Make sure the caught exceptions have the same types as the thrown exception
  // (This version copies the exception to a new object with the parent type)
  nav_core2::PlannerException e("");
  try
  {
    throw nav_core2::GlobalPlannerTimeoutException("test case");
  }
  catch (nav_core2::GlobalPlannerTimeoutException& x)
  {
    e = x;
  }
  EXPECT_EQ(e.getResultCode(), 12);
}

TEST(Exceptions, rethrow)
{
  // This version tests the ability to catch specific exceptions when rethrown
  // A copy of the exception is made and rethrown, with the goal being able to catch the specific type
  // and not the parent type.
  nav_core2::NavCore2ExceptionPtr e;
  try
  {
    throw nav_core2::GlobalPlannerTimeoutException("test case");
  }
  catch (nav_core2::GlobalPlannerTimeoutException& x)
  {
    e = std::current_exception();
  }

  EXPECT_EQ(nav_core2::getResultCode(e), 12);

  try
  {
    std::rethrow_exception(e);
  }
  catch (nav_core2::GlobalPlannerTimeoutException& x)
  {
    EXPECT_EQ(x.getResultCode(), 12);
    SUCCEED();
  }
  catch (nav_core2::PlannerException& x)
  {
    FAIL() << "PlannerException caught instead of specific exception";
  }
}

TEST(Exceptions, weird_exception)
{
  nav_core2::NavCore2ExceptionPtr e;

  // Check what happens with no exception
  EXPECT_EQ(nav_core2::getResultCode(e), -1);

  // Check what happens with a non NavCore2Exception
  try
  {
    std::string().at(1);  // this generates an std::out_of_range
  }
  catch (...)
  {
    e = std::current_exception();
  }

  EXPECT_EQ(nav_core2::getResultCode(e), -1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
