This is an Arduino program that takes inputs from an amplified filtered (low-pass ~30Hz) EOG signal, and detects when the user looks
up or down or blinks. It uses wide and narrow windows to detect transitions in the EOG signals.

It was a quick experiment to see whether cheap skin electrodes around the eye can reliably detect eye movements (moderately successful). The skin electrodes were not good enough and reliable signal acquisition was sporadic, but the code worked fine...
