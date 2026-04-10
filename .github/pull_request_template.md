## Summary
<!-- What changed and why? 1-3 bullet points -->

## Story
<!-- Link to the GitHub Issue this PR addresses -->
Closes #

## Test Plan
- [ ] Builds successfully (`cmake --build build`)
- [ ] Tested on Jetson Nano 2GB / WSL2 cross-compile (circle one)
- [ ] Gazebo simulation tested (if applicable)
- [ ] No memory regression (`tegrastats` checked, if applicable)

## Checklist
- [ ] Code follows project conventions (snake_case, no exceptions, struct-oriented)
- [ ] CUDA kernels: register usage checked (`--ptxas-options=-v`)
- [ ] No new heap allocations in hot loops
- [ ] Docs updated (if public interface changed)
