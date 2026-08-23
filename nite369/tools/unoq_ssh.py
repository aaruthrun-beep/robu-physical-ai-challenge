import paramiko
import sys
import os

HOST = "192.168.29.12"
PASSWD = "swarm@123"
USER = "arduino"

def run_cmd(client, cmd):
    """Run a command and return (exit_code, stdout, stderr)."""
    stdin, stdout, stderr = client.exec_command(cmd)
    exit_code = stdout.channel.recv_exit_status()
    return exit_code, stdout.read().decode("utf-8", errors="replace").strip(), stderr.read().decode("utf-8", errors="replace").strip()

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "echo 'Hello from swarmiji!' && hostname"

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASSWD, timeout=10,
                   allow_agent=False, look_for_keys=False)

    exit_code, out, err = run_cmd(client, cmd)
    # Write to file to avoid encoding issues on Windows console
    out_file = os.path.join(os.path.dirname(__file__), "_unoq_out.txt")
    with open(out_file, "w", encoding="utf-8") as f:
        if out:
            f.write(out + "\n")
        if err:
            f.write("STDERR:\n" + err + "\n")
    # Also print with replace
    if out:
        sys.stdout.buffer.write((out + "\n").encode("utf-8", errors="replace"))
    if err:
        sys.stderr.buffer.write(("STDERR:\n" + err + "\n").encode("utf-8", errors="replace"))
    client.close()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
