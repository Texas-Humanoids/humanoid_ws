"""mjlab's `list_envs` CLI with our tasks registered. Same flags as mjlab's `list_envs`."""
import th_training.tasks  # noqa: F401  (registers TH-* tasks)
from mjlab.scripts.list_envs import main

if __name__ == "__main__":
    main()
