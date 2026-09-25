# Turtlesim setup — install ROS 2 + turtlesim from scratch

A step-by-step guide to get **`turtlesim`** running. Turtlesim is the classic
ROS 2 "hello world": a little window with a turtle you can drive around with the
arrow keys. It has one job here — **prove that ROS 2 and the GUI display both
work** before you move on to the heavier course simulator (`SIM_RUNBOOK.md`).

**Target setup:** Windows 11 + WSL2 + Ubuntu 22.04 + **ROS 2 Humble**.
That matches our course project. If you're on a native Ubuntu 22.04 machine,
**skip Part 1** and start at Part 2 — everything else is identical.

> Turtlesim runs directly in WSL/Ubuntu. **No Docker, no amd64 emulation** —
> that stuff is only for the course container. This is the light, fast check.

---

## What you'll end up with

- ROS 2 Humble installed and available in every new terminal
- A turtlesim window on screen
- Arrow-key control of the turtle in a second terminal

Total time: ~20–40 min, mostly waiting on downloads.

---

## Before you start

- **Windows 11** (WSLg GUI support is built in — this is what makes the
  turtle window appear). Windows 10 works too but needs `wsl --update`.
- A working internet connection.
- ~5 GB free disk space for `ros-humble-desktop`.

---

## Part 1 — (Windows only) Install WSL2 + Ubuntu 22.04

*Skip this whole part if you already have Ubuntu 22.04 in WSL, or you're on a
native Ubuntu machine.*

1. Open **PowerShell as Administrator** (Start → type "PowerShell" → right-click
   → *Run as administrator*) and run:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```
2. **Reboot** if it asks you to.
3. After reboot, Ubuntu launches and asks you to create a **username and
   password**. Pick anything and remember the password — you'll type it for
   `sudo`.
4. Make sure WSL and WSLg are current (this is what draws GUI windows):
   ```powershell
   wsl --update
   wsl --shutdown
   ```
   Then reopen Ubuntu from the Start menu.

> **How to open your Ubuntu terminal later:** Start menu → "Ubuntu", or run
> `wsl` in any PowerShell/Command Prompt window. Your prompt looks like
> `youruser@MACHINE:~$`.

Everything from here on runs **inside the Ubuntu terminal**, not PowerShell.

---

## Part 2 — Set the locale

ROS 2 wants a UTF-8 locale. Set it once:

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Check it worked:
```bash
locale    # should show en_US.UTF-8 in the LANG/LC_* lines
```

---

## Part 3 — Add the ROS 2 apt repository

1. Enable the Ubuntu "universe" repo:
   ```bash
   sudo apt install -y software-properties-common
   sudo add-apt-repository -y universe
   ```
2. Add the ROS 2 GPG key:
   ```bash
   sudo apt update && sudo apt install -y curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```
3. Add the repository to your apt sources:
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

---

## Part 4 — Install ROS 2 Humble

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

`ros-humble-desktop` is the full install (includes RViz, demos, and turtlesim).
This is the big download — grab a coffee.

> Tight on disk? `ros-humble-ros-base` is the minimal version, but then you
> **must** also `sudo apt install -y ros-humble-turtlesim` in Part 6.

---

## Part 5 — Make ROS available in every terminal

ROS 2 lives under `/opt/ros/humble` and has to be "sourced" in each shell. So it
happens automatically, add it to your `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Confirm it's active:
```bash
printenv ROS_DISTRO        # should print: humble
ros2 --help                # should print ros2 command help
```

> If `ros2` says "command not found", the source line didn't run — open a fresh
> terminal, or run `source /opt/ros/humble/setup.bash` by hand.

---

## Part 6 — Install turtlesim (and handy dev tools)

Turtlesim ships with the desktop install, but installing it explicitly is
harmless and guarantees it's there:

```bash
sudo apt install -y ros-humble-turtlesim
```

Optional but recommended — the tools you'll want later for the course project
(`colcon`, `rosdep`, etc.):
```bash
sudo apt install -y ros-dev-tools
```

Verify turtlesim is found:
```bash
ros2 pkg executables turtlesim
```
Expect to see `turtlesim draw_square`, `turtlesim turtlesim_node`,
`turtlesim turtle_teleop_key`, and a couple others.

---

## Part 7 — Run turtlesim (the moment of truth)

In your Ubuntu terminal:
```bash
ros2 run turtlesim turtlesim_node
```

**Expect:** a blue window opens with a turtle in the middle, and the terminal
prints something like `Starting turtlesim with node name /turtlesim`.

🎉 If the window appears, **ROS 2 and your GUI display both work.** Leave this
terminal running.

*(If nothing appears, jump to Troubleshooting below.)*

---

## Part 8 — Drive the turtle

Open a **second** Ubuntu terminal (Start → Ubuntu again, or split your terminal)
and run:
```bash
ros2 run turtlesim turtle_teleop_key
```

Click that second terminal to focus it, then use the **arrow keys** to drive the
turtle around. You'll see it move and leave a trail in the first window.

> The arrow keys only work while the `turtle_teleop_key` terminal is the focused
> window. If the turtle won't move, click that terminal and try again.

That's a complete working ROS 2 install. ✅

---

## Troubleshooting

**The turtlesim window never appears / `qt.qpa.plugin: could not connect to
display` / `could not connect to display :0`**
This is the GUI/WSLg path. Fixes, in order:
1. From **PowerShell** (not Ubuntu): `wsl --update` then `wsl --shutdown`, then
   reopen Ubuntu. WSLg is what renders the window.
2. Confirm you're actually on WSL**2**: in PowerShell run `wsl -l -v` — the
   VERSION column must say `2`. If it says `1`:
   `wsl --set-version Ubuntu-22.04 2`.
3. Sanity-check the display with a tiny GUI app:
   ```bash
   sudo apt install -y x11-apps
   xeyes            # a pair of eyes should pop up; Ctrl-C to close
   ```
   If `xeyes` fails too, it's a WSLg problem, not a ROS problem — focus on
   step 1.

**`ros2: command not found`**
The setup file isn't sourced in this terminal. Run
`source /opt/ros/humble/setup.bash`, and make sure Part 5 added it to
`~/.bashrc` (open a brand-new terminal to test).

**`Unable to locate package ros-humble-desktop`**
The apt repo/key from Part 3 didn't take. Re-run Part 3, then `sudo apt update`.
Also double-check you're on Ubuntu 22.04 (`lsb_release -a` → `jammy`); Humble
targets 22.04.

**apt key / GPG errors during `apt update`**
Re-run the `curl ... ros.key` command in Part 3 step 2 (it must finish without
error), then `sudo apt update` again.

**The two terminals don't seem to talk to each other**
For turtlesim on one machine this is rare. If it happens, make sure both
terminals have ROS sourced and the same domain:
`echo $ROS_DOMAIN_ID` in both (blank/equal is fine). Don't set
`ROS_LOCALHOST_ONLY` for this test.

---

## Next step: the real course simulator

Turtlesim only proves your ROS 2 + GUI stack is healthy. The actual hockey
project runs a different, heavier simulator inside a Docker container (amd64
emulated, matplotlib GUI). When turtlesim works, move on to **`SIM_RUNBOOK.md`**
in this folder for that setup.

Key differences to expect there (not needed for turtlesim):
- It runs **inside a Docker container**, launched via `run_container.sh`.
- It needs the WSL **networking mode** set correctly (NAT at home, mirrored in
  the lab) — see the top of `SIM_RUNBOOK.md`.
- First launch is slow under emulation; that's normal.
