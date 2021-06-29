#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;
using namespace lely;


// This driver inherits from FiberDriver, which means that all CANopen event
// callbacks, such as OnBoot, run as a task inside a "fiber" (or stackful
// coroutine).
class MyDriver : public canopen::FiberDriver {
public:
    using FiberDriver::FiberDriver;

private:
    // This function gets called when the boot-up process of the slave completes.
    // The 'st' parameter contains the last known NMT state of the slave
    // (typically pre-operational), 'es' the error code (0 on success), and 'what'
    // a description of the error, if any.
    void
    OnBoot(canopen::NmtState /*st*/, char es,
           const std::string &what) noexcept override {
        if (!es || es == 'L') {
            std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully"
                      << std::endl;
        } else {
            std::cout << "slave " << static_cast<int>(id())
                      << " failed to boot: " << what << std::endl;
        }
    }

    void OnHeartbeat(bool occurred) noexcept override {
        std::cout << "heartbeat " << occurred << std::endl;
    }


    // This function gets called during the boot-up process for the slave. The
    // 'res' parameter is the function that MUST be invoked when the configuration
    // is complete. Because this function runs as a task inside a coroutine, it
    // can suspend itself and wait for an asynchronous function, such as an SDO
    // request, to complete.
    void
    OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {

            uint32_t sn = Wait(AsyncRead<uint32_t>(0x1018, 3));
            std::cout << "slave S/N " << static_cast<int>(sn) << std::endl;

            // Report success (empty error code).
            res({});
        } catch (canopen::SdoError &e) {
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
        }
    }

    // This function gets called every time a value is written to the local object
    // dictionary of the master by an RPDO (or SDO, but that is unlikely for a
    // master), *and* the object has a known mapping to an object on the slave for
    // which this class is the driver. The 'idx' and 'subidx' parameters are the
    // object index and sub-index of the object on the slave, not the local object
    // dictionary of the master.
    void
    OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override {


        if (idx == 0x6000 && subidx == 1) {
            // Obtain the value sent by PDO from object 4001:00 on the slave.
            uint8_t val = rpdo_mapped[0x6000][1];
            std::cout << "RPDO " << static_cast<int>(val) << std::endl;
        }
    }

    void
    OnSync(uint8_t cnt, const time_point &t) noexcept override {
        std::cout << "sync " << std::to_string(std::chrono::duration<double>(t.time_since_epoch()).count())
                  << std::endl;
    }

};


int
main() {
    // Initialize the I/O library. This is required on Windows, but a no-op on
    // Linux (for now).
    io::IoGuard io_guard;
    // Create an I/O context to synchronize I/O services during shutdown.
    io::Context ctx;
    // Create an platform-specific I/O polling instance to monitor the CAN bus, as
    // well as timers and signals.
    io::Poll poll(ctx);
    // Create a polling event loop and pass it the platform-independent polling
    // interface. If no tasks are pending, the event loop will poll for I/O
    // events.
    ev::Loop loop(poll.get_poll());
    // I/O devices only need access to the executor interface of the event loop.
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);

    // Create a virtual SocketCAN CAN controller and channel, and do not modify
    // the current CAN bus state or bitrate.
    io::CanController ctrl("vcan0");
    io::CanChannel chan(poll, exec);

    chan.open(ctrl);

    // means every user-defined callback for a CANopen event will be posted as a
    // task on the event loop, instead of being invoked during the event
    // processing by the stack.
    canopen::AsyncMaster master(timer, chan, "../canopen-401-force-sync-1tpdo.dcf", "", 1);

    // Create a driver for the slave with node-ID 2.
    MyDriver driver(exec, master, 2);

    // Create a signal handler.
    io::SignalSet sigset(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
        // If the signal is raised again, terminate immediately.
        sigset.clear();

        // Tell the master to start the deconfiguration process for all nodes, and
        // submit a task to be executed once that process completes.
        master.AsyncDeconfig().submit(exec, [&]() {
            // Perform a clean shutdown.
            ctx.shutdown();
        });
    });

    // Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    master.Reset();

    // Run the event loop until no tasks remain (or the I/O context is shut down).
    loop.run();

    return 0;
}