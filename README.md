# LibFPrint Elanpress for 04f3:0c6e

This is a personal fork of mine made after capturing some fingerprint packets
from the windows driver to attempt to get the sensor functional on linux. It is
in a working state and I use it daily.

The project is however not finished and I will keep maintaining it until I have
something that works well. Pull requests and issues are much appreciated.

As for LLM usage when submitting pull requests; I do not have any issue with the
code being written by LLM's but it's most preferable if you atleast test the code
on your machine before opening any PR. You are responsible for the PR even if
an agent makes it and I will close PR's without further review if they do not seem
up to a reasonable standard.

My only test machine for this sensor is an ASUS ROG Flow X13 GV301RC, so if you
have any other machine with the same sensor and are able to test that would be very
very nice.

## Installing to test

First check that you have the sensor:

```sh
lsusb | grep 04f3:0c6e
```

This replaces your distro's libfprint, so fprintd, GDM and the lockscreen
all use it. Old prints won't carry over, so you have
to enroll your fingers again after installing.

### Arch Linux

The driver is on the AUR as
[libfprint-elanpress-git](https://aur.archlinux.org/packages/libfprint-elanpress-git),
which builds the latest commit of this branch and replaces `libfprint`:

```sh
paru -S libfprint-elanpress-git   # or any other AUR helper
```

Reinstall it to pick up new commits. To go back, install `libfprint` again.

### Other distributions

Install libfprint's build dependencies (`sudo dnf builddep libfprint` on Fedora,
`sudo apt build-dep libfprint` on Debian/Ubuntu with source repositories
enabled), then build and install over the system library:

```sh
git clone -b elanpress https://github.com/filip-rs/libfprint.git
cd libfprint
meson setup build --prefix=/usr -Ddoc=false -Dinstalled-tests=false
meson compile -C build
sudo meson install -C build
```

Your package manager doesn't know about these files, so a libfprint update from
your distribution will overwrite them. Reinstalling your distribution's
libfprint package also restores the official upstream package.

### Enrolling and testing

Restart fprintd so it loads the new library, then enroll and verify:

```sh
sudo systemctl restart fprintd
fprintd-enroll -f right-index-finger
fprintd-verify -f right-index-finger
```

Enrolling takes several separate presses. Press the pad firmly and cover it
properly as a light or partial touch is a common reason the enrolling fails.

If you want to report a problem, the driver logs every press with its match score
(`NCC`) and coverage. To turn those logs on, run `sudo systemctl edit fprintd`
and add:

```ini
[Service]
Environment=G_MESSAGES_DEBUG=all
```

Then restart fprintd and read the log with `journalctl -u fprintd`. Scores from
both the right finger and other fingers is really helpful for tuning the threshold.

<div align="center">

_LibFPrint is part of the **[FPrint][Website]** project._

<br/>

[![Button Website]][Website]
[![Button Documentation]][Documentation]

[![Button Supported]][Supported]
[![Button Unsupported]][Unsupported]

[![Button Contribute]][Contribute]
[![Button Contributors]][Contributors]

</div>

## History

**LibFPrint** was originally developed as part of an
academic project at the **[University Of Manchester]**.

It aimed to hide the differences between consumer
fingerprint scanners and provide a single uniform
API to application developers.

## Goal

The ultimate goal of the **FPrint** project is to make
fingerprint scanners widely and easily usable under
common Linux environments.

## License

`Section 6` of the license states that for compiled works that use
this library, such works must include **LibFPrint** copyright notices
alongside the copyright notices for the other parts of the work.

**LibFPrint** includes code from **NIST's** **[NBIS]** software distribution.

We include **Bozorth3** from the **[US Export Controlled]**
distribution, which we have determined to be fine
being shipped in an open source project.

## Get in _touch_

- [IRC] - `#fprint` @ `irc.oftc.net`
- [Matrix] - `#fprint:matrix.org` bridged to the IRC channel
- [MailingList] - low traffic, not much used these days

<br/>

<div align="right">

[![Badge License]][License]

</div>

<!----------------------------------------------------------------------------->

[Documentation]: https://fprint.freedesktop.org/libfprint-dev/
[Contributors]: https://gitlab.freedesktop.org/libfprint/libfprint/-/graphs/master
[Unsupported]: https://gitlab.freedesktop.org/libfprint/wiki/-/wikis/Unsupported-Devices
[Supported]: https://fprint.freedesktop.org/supported-devices.html
[Website]: https://fprint.freedesktop.org/
[MailingList]: https://lists.freedesktop.org/mailman/listinfo/fprint
[IRC]: ircs://irc.oftc.net:6697/#fprint
[Matrix]: https://matrix.to/#/#fprint:matrix.org
[Contribute]: ./HACKING.md
[License]: ./COPYING
[University Of Manchester]: https://www.manchester.ac.uk/
[US Export Controlled]: https://fprint.freedesktop.org/us-export-control.html
[NBIS]: http://fingerprint.nist.gov/NBIS/index.html

<!---------------------------------[ Badges ]---------------------------------->

[Badge License]: https://img.shields.io/badge/License-LGPL2.1-015d93.svg?style=for-the-badge&labelColor=blue

<!---------------------------------[ Buttons ]--------------------------------->

[Button Documentation]: https://img.shields.io/badge/Documentation-04ACE6?style=for-the-badge&logoColor=white&logo=BookStack
[Button Contributors]: https://img.shields.io/badge/Contributors-FF4F8B?style=for-the-badge&logoColor=white&logo=ActiGraph
[Button Unsupported]: https://img.shields.io/badge/Unsupported_Devices-EF2D5E?style=for-the-badge&logoColor=white&logo=AdBlock
[Button Contribute]: https://img.shields.io/badge/Contribute-66459B?style=for-the-badge&logoColor=white&logo=Git
[Button Supported]: https://img.shields.io/badge/Supported_Devices-428813?style=for-the-badge&logoColor=white&logo=AdGuard
[Button Website]: https://img.shields.io/badge/Homepage-3B80AE?style=for-the-badge&logoColor=white&logo=freedesktopDotOrg
