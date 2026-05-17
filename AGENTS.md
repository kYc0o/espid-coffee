# AGENTS.md — handoff & conventions

If you are an AI coding agent (or a new human contributor) picking up
this project, **read this entire file before making changes**.
Companion: [`ARCHITECTURE.md`](ARCHITECTURE.md) for the structural
view of the system.

---

## 1. Project in one minute

ESP-IDF v6.0.x firmware for the Rancilio Silvia espresso machine,
targeting **ESP32-S3 N16R8**. Replaces the Arduino-framework
[CleverCoffee](https://github.com/rancilio-pid/clevercoffee) with a
modern C++17 / FreeRTOS native / TDD codebase. The user is also the
project owner and runs the firmware on real hardware between turns.

Owner / workstation: macOS, zsh, ESP-IDF cloned at
`/Users/facosta/git/espressif/esp-idf-v6.0.1`. Device on
`/dev/cu.usbmodem5ABA0676261`.

---

## 2. Hard rules — non-negotiable

1. **Don't push** unless the user explicitly says so. Local commits
   are fine. `git push` (and any other shared-state mutation) requires
   user confirmation each time.
2. **Bisectability is sacred.** Every commit on `main` must build
   _and_ pass host tests in isolation. Validate with
   `scripts/bisect_verify.sh -n N` before considering work done.
3. **Atomic Conventional-Commits commits.** One logical change per
   commit, subject ≤ 72 chars, type and scope mandatory. Body via
   `git commit -F /tmp/cN.txt` heredoc — never inline `-m` for
   anything non-trivial.
4. **C++17, no exceptions, no RTTI.** Use `Result<T>`/`Status` for
   error propagation. Both are `[[nodiscard]]`.
5. **No magic numbers in firmware code.** Add them to
   [`main/config.hpp`](main/config.hpp) first, reference from there.
6. **TDD for pure logic.** Anything that doesn't directly touch
   ESP-IDF gets host tests in `host_tests/`. PID, decoder, brew
   button, shot timer all follow this. Aim for green host tests
   before host-target wiring.
7. **HAL headers must compile on the Linux target.** No ESP-IDF
   types in `components/hal_cpp/include/hal_cpp/*.hpp`. If a HAL
   header includes `driver/gpio.h`, that's a bug.

---

## 3. Build & flash commands

Use the ESP-IDF shell setup once per terminal session, then call
`idf.py` normally.

```sh
# Preferred (alias from ~/.zshrc)
get_idf

# Fallback (if alias unavailable)
. /Users/facosta/git/espressif/esp-idf-v6.0.1/export.sh
```

Common `<CMD>` values:

| Goal              | Command                                              |
| ----------------- | ---------------------------------------------------- |
| Build firmware    | `idf.py build`                                       |
| Flash + monitor   | `idf.py -p /dev/cu.usbmodem5ABA0676261 flash monitor`|
| Flash only        | `idf.py -p /dev/cu.usbmodem5ABA0676261 flash`        |
| Reconfigure menus | `idf.py menuconfig`                                  |
| Erase             | `idf.py -p /dev/cu.usbmodem5ABA0676261 erase-flash`  |

Host tests (no env scrub needed — uses system CMake):

```sh
cmake -S host_tests -B host_tests/build
cmake --build host_tests/build
ctest --test-dir host_tests/build --output-on-failure
```

Bisect verification (always runs `idf.py build` + host tests on each
commit, then restores HEAD):

```sh
scripts/bisect_verify.sh -n 3      # last 3 commits
scripts/bisect_verify.sh -s <sha>  # since <sha> exclusive
scripts/bisect_verify.sh           # all commits in branch
```

---

## 4. Commit etiquette

Format:

```
<type>(<scope>): <subject ≤ 72 chars>

<wrapped body — what & why, not how; reference design constraints,
trade-offs, and any visible behaviour change>
```

Types in use: `feat`, `fix`, `chore`, `build`, `refactor`, `test`,
`docs`. Scopes are component-or-area names: `tsic306`, `ssd1306`,
`brew`, `pid`, `hal_cpp`, `main`, `scripts`, `host-tests`, …

Body workflow (avoids quoting hell):

```sh
cat > /tmp/cN.txt <<'EOF'
feat(scope): subject

Body paragraphs here.
EOF
git add <paths>
git commit -F /tmp/cN.txt
```

Never `--amend` published commits. Never `--force-push`. Never bypass
hooks with `--no-verify`.

---

## 5. Coding conventions

- C++17, `-Wall -Wextra` clean, no unused-warning suppressions
  except for FreeRTOS task argument plumbing (`void* /*arg*/`).
- `namespace espid::<component>` for every public symbol. Anonymous
  namespace inside `.cpp` for translation-unit-private helpers.
- `constexpr` over `#define`. `enum class` over plain `enum`.
- `static_cast` over C-style casts; no `reinterpret_cast` except in
  driver glue.
- `std::atomic<T>` for cross-task scalars, with explicit memory
  orderings (`release` on producer write of `valid`, `acquire` on
  consumer read).
- One class per `.hpp`, header guard via `#pragma once`.
- File header: short purpose comment, lifecycle contract, threading
  notes (see existing files for the template).
- `.clang-format` is at the repo root — keep diffs format-clean.

---

## 6. Adding a new component

```sh
mkdir -p components/<name>/{include/<name>,src}
```

Component CMake (header-only):

```cmake
idf_component_register(INCLUDE_DIRS "include")
```

With sources:

```cmake
idf_component_register(
    SRCS "src/<name>.cpp"
    INCLUDE_DIRS "include"
    REQUIRES <idf deps>
    PRIV_REQUIRES <internal deps>
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>)
```

To consume in firmware: add to `main/CMakeLists.txt` `REQUIRES`.

If the component has pure logic, mirror it in `host_tests/`:
1. Add an `INTERFACE` (header-only) or `STATIC` library target in
   [`host_tests/CMakeLists.txt`](host_tests/CMakeLists.txt).
2. Link it into `espid_unit_tests`.
3. Add a `test_<name>.cpp` next to the existing test files.

---

## 7. Operational safety

| Action                       | Allowed?                                  |
| ---------------------------- | ----------------------------------------- |
| `idf.py build`               | Always                                    |
| `idf.py flash`               | Yes, on the user's device                 |
| Editing files                | Yes                                       |
| `git commit` locally         | Yes, atomically                           |
| `git push`                   | **Ask first, every time**                 |
| `git reset --hard`           | **Ask first**                             |
| `git rebase` on `main`       | **Ask first**                             |
| `idf.py erase-flash`         | **Ask first**                             |
| Editing managed_components   | Avoid; patch via top-level CMakeLists     |
| Adding a managed dep         | Discuss the licence/maintenance burden    |

---

## 8. Where to look first

- **What is the firmware doing right now?** → [`main/app_main.cpp`](main/app_main.cpp)
- **What are the boot-time defaults?** → [`main/config.hpp`](main/config.hpp)
- **What's the data flow?** → [`ARCHITECTURE.md`](ARCHITECTURE.md) §7
- **Which tasks exist, with what stack/prio?** → [`ARCHITECTURE.md`](ARCHITECTURE.md) §5
- **What's the next phase?** → [`ARCHITECTURE.md`](ARCHITECTURE.md) §12 roadmap
- **How do I run host tests?** → §3 of this file
- **Error vocabulary** → [`components/hal_cpp/include/hal_cpp/result.hpp`](components/hal_cpp/include/hal_cpp/result.hpp)
- **HAL contracts** → [`components/hal_cpp/include/hal_cpp/`](components/hal_cpp/include/hal_cpp)

---

## 9. Known gotchas

- **nixy4/u8g2 v0.1.4 + ESP-IDF v6** — the managed component is
  missing `esp_driver_gpio` in its `REQUIRES`. The top-level
  [`CMakeLists.txt`](CMakeLists.txt) patches its component manifest
  in place. Don't remove that patch.
- **`Rotation` enum scope** — `Rotation` is a sibling member of
  `Ssd1306Display`, _not_ nested inside `Config`. Refer to it as
  `Ssd1306Display::Rotation::k180`.
- **OLED non-critical** — if init fails we continue without UI; don't
  hard-halt on OLED failures.
- **TSIC sensor frames arrive every ~100 ms** — splash duration must
  be a fixed delay, not a "wait until first sample" guard.
- **GPIO 5 brew button** — already RC + Schmitt-trigger debounced in
  hardware; the 10 ms software debounce is belt-and-suspenders.
  Active-low.

---

## 10. Working with the user

- The user owns the hardware and tests every flash personally. After
  any firmware change that affects visible behaviour, expect a manual
  verification step before committing.
- Iterate small. The user prefers many small atomic commits to one
  large one.
- Don't ask questions you can answer by reading the codebase or by
  trying the obvious sensible default.
- Always offer the next phase explicitly once a milestone lands.
