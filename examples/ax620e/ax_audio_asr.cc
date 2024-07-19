#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <signal.h>
#include <unistd.h>
#include <vector>
#include <sys/types.h>
#include <sys/wait.h>

std::vector<pid_t> child_pids;

bool stop = false;

// static void Handler(int sig) {
//     stop = true;
//     fprintf(stderr, "\nCaught Ctrl + C. Exiting...\n");
// }

void run_audio_input() {
    std::string command = "LD_LIBRARY_PATH=/root/ax-audio/install/lib /root/ax-audio/install/bin/ax_audio_input ai -D 0 -d 0 -r 16000 -p 160 --layout 2 --aenc-chns 2 -w 1";
    pid_t pid = fork();
    if (pid == 0) {
        // 子进程
        std::system(command.c_str());
        exit(0);
    } else if (pid > 0) {
        // 父进程
        child_pids.push_back(pid);
    } else {
        std::cerr << "Fork failed" << std::endl;
    }
}

void run_sherpa_asr() {
    std::string command = "/root/sherpa/install/bin/sherpa-onnx --tokens=/root/sherpa/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/tokens.txt --encoder=/root/sherpa/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/encoder-epoch-99-avg-1.int8.onnx --decoder=/root/sherpa/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/decoder-epoch-99-avg-1.int8.onnx --joiner=/root/sherpa/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/joiner-epoch-99-avg-1.int8.onnx audio.wav";
    pid_t pid = fork();
    if (pid == 0) {
        // 子进程
        std::system(command.c_str());
        exit(0);
    } else if (pid > 0) {
        // 父进程
        child_pids.push_back(pid);
    } else {
        std::cerr << "Fork failed" << std::endl;
    }
}

void stop_audio_input() {
    for (pid_t pid : child_pids) {
        if (kill(pid, SIGTERM) == 0) {
            int status;
            waitpid(pid, &status, 0);
            std::cout << "Process " << pid << " terminated." << std::endl;
        } else {
            std::cerr << "Failed to terminate process " << pid << std::endl;
        }
    }
    child_pids.clear();
}

void stop_sherpa_asr() {
    for (pid_t pid : child_pids) {
        if (kill(pid, SIGTERM) == 0) {
            int status;
            waitpid(pid, &status, 0);
            std::cout << "Process " << pid << " terminated." << std::endl;
        } else {
            std::cerr << "Failed to terminate process " << pid << std::endl;
        }
    }
    child_pids.clear();
}

void stop_processes() {
    stop_audio_input();
    stop_sherpa_asr();
}


int main(int argc, char* argv[]) {
    // signal(SIGINT, Handler);
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <runtime_in_seconds>" << std::endl;
        return 1;
    }
    int runtime = std::stoi(argv[1]);
    std::cout << "Running ./ax_audio_input ..." << std::endl;
    run_audio_input();

    std::this_thread::sleep_for(std::chrono::seconds(runtime));
    stop_audio_input();

    std::cout << "Running ./sherpa-onnx ..." << std::endl;
    run_sherpa_asr();

    // while (!stop) {
    //
    // }

    if (stop) {
        std::cout << "Stopping ./ax_audio_asr ..." << std::endl;
        stop_processes();
    }

    return 0;
}

