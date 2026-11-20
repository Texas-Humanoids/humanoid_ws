"""mjlab's `train` CLI with our tasks registered. Same flags as mjlab's `train`."""
import th_training.tasks  # noqa: F401  (registers TH-* tasks)
from mjlab.scripts.train import main

if __name__ == "__main__":
    main()
