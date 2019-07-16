
#include <signal.h>
#include <cstddef>

static bool exit_app = false;

// Handle the CTRL-C keyboard signal

#include <signal.h>
void nix_exit_handler(int s) {
    exit_app = true;
}


// Set the function to handle the CTRL-C
void SetCtrlHandler() {

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = nix_exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

}
