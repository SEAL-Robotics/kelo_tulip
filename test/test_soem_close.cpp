#include <gtest/gtest.h>

#include <cstring>
#include <fcntl.h>
#include <unistd.h>

extern "C" {
#include "kelo_tulip/soem/ethercattype.h"
#include "nicdrv.h"
}

namespace {

// Closing the port twice must not close whatever now owns the old fd number.
TEST(EcxClosenic, aSecondCloseLeavesAReusedFdAlone) {
	ecx_portt port;
	std::memset(&port, 0, sizeof(port));
	port.redport = nullptr;
	port.sockhandle = open("/dev/null", O_RDONLY);
	ASSERT_GE(port.sockhandle, 0);

	ecx_closenic(&port);
	const int reused = open("/dev/null", O_RDONLY);  // takes the freed number
	ecx_closenic(&port);

	EXPECT_NE(fcntl(reused, F_GETFD), -1) << "the second close hit an fd it no longer owns";
	close(reused);
}

}  // namespace
