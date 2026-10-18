"""CPU and GPU utilization sampled while an experiment integrates."""

import os
import statistics
import subprocess
import threading


class Utilization:
    """Samples this process's CPU use and one GPU's utilization while active.

    CPU is reported in cores (100% of one core = 1) for this process and as a
    percentage of all cores for the machine. GPU utilization and memory are
    device-wide, so other processes on the same device are included; the
    number of compute processes on it (this one included) is recorded alongside.
    """

    INTERVAL = .25

    def __init__(self, gpu):
        import psutil
        self.gpu, self.process = gpu, psutil.Process()
        self.active, self.samples = False, []
        self.stopped = threading.Event()
        self.gpu_reading = None
        self.smi = subprocess.Popen(
            ['nvidia-smi', '-i', str(gpu), '--query-gpu=utilization.gpu,memory.used',
             '--format=csv,noheader,nounits', f'-lms={int(self.INTERVAL*1000)}'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        threading.Thread(target=self.read_gpu, daemon=True).start()
        threading.Thread(target=self.sample, daemon=True).start()

    def read_gpu(self):
        for line in self.smi.stdout:
            try:
                self.gpu_reading = tuple(float(value) for value in line.split(','))
            except ValueError:
                pass

    def sample(self):
        import psutil
        self.process.cpu_percent()
        psutil.cpu_percent()
        while not self.stopped.wait(self.INTERVAL):
            process, machine = self.process.cpu_percent(), psutil.cpu_percent()
            if self.active and self.gpu_reading:
                self.samples.append((process / 100, machine, *self.gpu_reading))

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    def summary(self):
        """Mean and peak of each quantity over the active periods since the last summary."""
        self.active = False
        samples, self.samples = self.samples, []
        if not samples:
            return None
        columns = list(zip(*samples))
        stats = lambda values: {'mean': statistics.fmean(values), 'max': max(values)}
        processes = subprocess.run(['nvidia-smi', '-i', str(self.gpu), '--query-compute-apps=pid', '--format=csv,noheader'],
                                   capture_output=True, text=True).stdout.split()
        return {'samples': len(samples), 'interval_s': self.INTERVAL,
                'process_cpu_cores': stats(columns[0]), 'machine_cpu_percent': stats(columns[1]),
                'machine_cpu_count': os.cpu_count(),
                'gpu_physical_index': self.gpu, 'gpu_utilization_percent': stats(columns[2]),
                'gpu_memory_used_MiB': stats(columns[3]),
                'compute_processes_on_gpu': len([pid for pid in processes if pid.isdigit()]),
                'note': 'GPU utilization and memory are device-wide; the process count includes this one'}

    def close(self):
        self.stopped.set()
        self.smi.terminate()

