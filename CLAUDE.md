# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is the Pupper V3 monorepo - a quadruped robot with ROS2 control, neural network locomotion policies, and an AI voice assistant interface.

Explore the repository files to understand context. There aren't that many files.

## Coding guidelines

Never write fallbacks. Instead warn the user about potential failure modes. 

Use uv to manage packages. Never modify the PATH manually in files.