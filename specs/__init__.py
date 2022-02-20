from typing import Any

from numpy import allclose
from hamcrest.core.core.isequal import IsEqual


class NumpyCloseMatcher(IsEqual):
    def _matches(self, item: Any) -> bool:
        return allclose(item, self.object)


def np_close(obj: Any) -> NumpyCloseMatcher:
    return NumpyCloseMatcher(obj)
