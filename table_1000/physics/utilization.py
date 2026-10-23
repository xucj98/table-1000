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
        self.lock = threading.Lock()
        self.gpu_reading = None
        self.errors = []
        self.smi = None
        self.reader = None
        try:
            self.smi = subprocess.Popen(
                ['nvidia-smi', '-i', str(gpu), '--query-gpu=utilization.gpu,memory.used',
                 '--format=csv,noheader,nounits', f'-lms={int(self.INTERVAL*1000)}'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            self.reader = threading.Thread(target=self.read_gpu, name=f'gpu-reader-{gpu}', daemon=True)
            self.reader.start()
        except Exception as error:
            self.errors.append(f'GPU sampler unavailable: {type(error).__name__}: {error}')
        self.sampler = threading.Thread(target=self.sample, name=f'resource-sampler-{gpu}', daemon=True)
        self.sampler.start()

    def read_gpu(self):
        try:
            for line in self.smi.stdout:
                try:
                    reading=tuple(float(value) for value in line.split(','))
                    with self.lock:
                        self.gpu_reading=reading
                except ValueError:
                    continue
        except (OSError,ValueError) as error:
            if not self.stopped.is_set():
                self.errors.append(f'GPU sampler read failed: {type(error).__name__}: {error}')

    def sample(self):
        import psutil
        self.process.cpu_percent()
        psutil.cpu_percent()
        while not self.stopped.wait(self.INTERVAL):
            process, machine = self.process.cpu_percent(), psutil.cpu_percent()
            with self.lock:
                reading=self.gpu_reading
                if self.active and reading:
                    self.samples.append((process / 100, machine, *reading))

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    def summary(self):
        """Mean and peak of each quantity over the active periods since the last summary."""
        with self.lock:
            self.active = False
            samples, self.samples = self.samples, []
        columns = list(zip(*samples)) if samples else []
        stats = lambda values: {'mean': statistics.fmean(values), 'max': max(values)}
        processes=None
        try:
            query=subprocess.run(['nvidia-smi', '-i', str(self.gpu), '--query-compute-apps=pid', '--format=csv,noheader'],
                                 capture_output=True,text=True,timeout=5,check=True)
            processes=len([pid for pid in query.stdout.split() if pid.isdigit()])
        except Exception as error:
            self.errors.append(f'GPU process query unavailable: {type(error).__name__}: {error}')
        result={'status':'complete' if samples and processes is not None else 'partial' if samples or processes is not None else 'unavailable',
                'samples':len(samples),'interval_s':self.INTERVAL,
                'process_cpu_cores':stats(columns[0]) if samples else None,
                'machine_cpu_percent':stats(columns[1]) if samples else None,
                'machine_cpu_count':os.cpu_count(),
                'gpu_physical_index':self.gpu,
                'gpu_utilization_percent':stats(columns[2]) if samples else None,
                'gpu_memory_used_MiB':stats(columns[3]) if samples else None,
                'compute_processes_on_gpu':processes,
                'note':'GPU utilization and memory are device-wide; the process count includes this one'}
        if self.errors:
            result['errors']=list(self.errors)
        return result

    def close(self):
        if self.stopped.is_set():
            return self.close_status
        self.stopped.set()
        self.close_status={'status':'closed','child_pid':self.smi.pid if self.smi else None,
                           'child_reaped':self.smi is None}
        if self.smi is not None:
            try:
                self.smi.terminate()
                self.smi.wait(timeout=2)
                self.close_status['child_reaped']=True
            except subprocess.TimeoutExpired:
                self.close_status['status']='killed_after_timeout'
                self.smi.kill()
                try:
                    self.smi.wait(timeout=2)
                    self.close_status['child_reaped']=True
                except subprocess.TimeoutExpired:
                    self.close_status['child_reaped']=False
            except Exception as error:
                self.close_status={'status':'close_error','child_pid':self.smi.pid,
                                   'child_reaped':False,'error':f'{type(error).__name__}: {error}'}
            try:
                if self.smi.stdout:
                    self.smi.stdout.close()
            except OSError:
                pass
        if self.reader is not None:
            self.reader.join(timeout=2)
            self.close_status['reader_stopped']=not self.reader.is_alive()
        self.sampler.join(timeout=2)
        self.close_status['sampler_stopped']=not self.sampler.is_alive()
        return self.close_status
