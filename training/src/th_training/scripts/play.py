"""mjlab's `play` CLI with our tasks registered. Same flags as mjlab's `play`."""
import th_training.tasks  # noqa: F401  (registers TH-* tasks)
from mjlab.scripts.play import main

if __name__ == "__main__":
    main()
