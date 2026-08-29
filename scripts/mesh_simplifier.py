# /// script
# dependencies = [
#   "pymeshlab",
#   "loguru",
#   "tyro",
# ]
# ///
"""
STL Mesh Simplification CLI using PyMeshLab.

Based on code from https://github.com/Rhoban/onshape-to-robot

Reduces STL mesh complexity using quadric edge collapse decimation.
Intended for collision meshes; does not support visual meshes (DAE) that
contain materials/colors.

Usage:
    uv run mesh_simplifier.py --directory ./meshes --max-size 2.0
    uv run mesh_simplifier.py --directory ./meshes --ratio 0.5
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated

import pymeshlab
from loguru import logger as log


DEFAULT_MAX_SIZE_MB: float = 1.0
DEFAULT_QUALITY_THRESHOLD: float = 0.5
DEFAULT_BOUNDARY_WEIGHT: float = 1.0
DEFAULT_PLANAR_WEIGHT: float = 0.001


def _find_stl_files(directory: Path) -> list[Path]:
    """Find all STL files (case-insensitive)."""
    return list(directory.glob("**/*.[sS][tT][lL]"))


@dataclass(frozen=True)
class DecimationSettings:
    """
    Configuration for quadric edge collapse decimation.

    These settings control how the mesh simplification algorithm operates,
    balancing between mesh quality preservation and reduction ratio.
    """

    quality_threshold: float = DEFAULT_QUALITY_THRESHOLD
    preserve_boundary: bool = False
    boundary_weight: float = DEFAULT_BOUNDARY_WEIGHT
    preserve_normal: bool = True
    preserve_topology: bool = False
    optimal_placement: bool = True
    planar_quadric: bool = True
    quality_weight: bool = False
    planar_weight: float = DEFAULT_PLANAR_WEIGHT
    auto_clean: bool = True
    selected_only: bool = False


@dataclass
class SimplificationResult:
    """Result of a mesh simplification operation."""

    filepath: Path
    original_size_mb: float
    final_size_mb: float
    reduction_applied: float
    was_simplified: bool

    @property
    def size_reduction_percent(self) -> float:
        """Calculate percentage of size reduction."""
        if self.original_size_mb == 0:
            return 0.0
        return (1 - self.final_size_mb / self.original_size_mb) * 100


class MeshSimplifier:
    """
    Mesh simplification using PyMeshLab's quadric edge collapse algorithm.

    Example:
        >>> simplifier = MeshSimplifier()
        >>> result = simplifier.simplify_if_needed("model.stl", max_size_mb=2.0)
        >>> log.info(f"Reduced by {result.size_reduction_percent:.1f}%")
    """

    def __init__(self, settings: DecimationSettings | None = None) -> None:
        self._settings = settings or DecimationSettings()

    @property
    def settings(self) -> DecimationSettings:
        return self._settings

    @staticmethod
    def get_file_size_mb(filepath: str | Path) -> float:
        """Get file size in megabytes."""
        return os.path.getsize(filepath) / (1024 * 1024)

    def reduce_faces(self, filepath: str | Path, target_ratio: float = 0.9) -> None:
        """
        Reduce mesh faces using quadric edge collapse decimation.

        Args:
            filepath: Path to the STL file to simplify.
            target_ratio: Target ratio of faces to keep (0.0-1.0).
        """
        filepath = Path(filepath)
        log.debug(f"Loading mesh: {filepath}")

        mesh_set = pymeshlab.MeshSet()
        mesh_set.load_new_mesh(str(filepath))

        log.debug(f"Applying decimation with target ratio: {target_ratio}")
        mesh_set.apply_filter(
            "meshing_decimation_quadric_edge_collapse",
            targetperc=target_ratio,
            qualitythr=self._settings.quality_threshold,
            preserveboundary=self._settings.preserve_boundary,
            boundaryweight=self._settings.boundary_weight,
            preservenormal=self._settings.preserve_normal,
            preservetopology=self._settings.preserve_topology,
            optimalplacement=self._settings.optimal_placement,
            planarquadric=self._settings.planar_quadric,
            qualityweight=self._settings.quality_weight,
            planarweight=self._settings.planar_weight,
            autoclean=self._settings.auto_clean,
            selected=self._settings.selected_only,
        )

        mesh_set.save_current_mesh(str(filepath))
        log.debug(f"Saved simplified mesh: {filepath}")

    def simplify_if_needed(
        self,
        filepath: str | Path,
        max_size_mb: float = DEFAULT_MAX_SIZE_MB,
    ) -> SimplificationResult:
        """
        Simplify mesh only if it exceeds the size threshold.

        Args:
            filepath: Path to the STL file.
            max_size_mb: Maximum allowed file size in megabytes.

        Returns:
            SimplificationResult with details about the operation.
        """
        filepath = Path(filepath)
        original_size = self.get_file_size_mb(filepath)

        if original_size <= max_size_mb:
            log.debug(
                f"Skipping {filepath.name}: {original_size:.2f}MB <= {max_size_mb}MB"
            )
            return SimplificationResult(
                filepath=filepath,
                original_size_mb=original_size,
                final_size_mb=original_size,
                reduction_applied=1.0,
                was_simplified=False,
            )

        target_ratio = max_size_mb / original_size
        log.info(
            f"Simplifying {filepath.name}: {original_size:.2f}MB -> "
            f"target {max_size_mb:.2f}MB (ratio: {target_ratio:.2f})"
        )

        self.reduce_faces(filepath, target_ratio)
        final_size = self.get_file_size_mb(filepath)

        log.success(
            f"Simplified {filepath.name}: {original_size:.2f}MB -> {final_size:.2f}MB "
            f"({(1 - final_size / original_size) * 100:.1f}% reduction)"
        )

        return SimplificationResult(
            filepath=filepath,
            original_size_mb=original_size,
            final_size_mb=final_size,
            reduction_applied=target_ratio,
            was_simplified=True,
        )

    def process_directory(
        self,
        directory: str | Path,
        max_size_mb: float = DEFAULT_MAX_SIZE_MB,
    ) -> list[SimplificationResult]:
        """
        Process all STL files in a directory (recursively).

        Args:
            directory: Directory containing STL files.
            max_size_mb: Maximum allowed file size in megabytes.

        Returns:
            List of SimplificationResult for each processed file.
        """
        directory = Path(directory)
        stl_files = _find_stl_files(directory)

        log.info(f"Found {len(stl_files)} STL file(s) in {directory}")

        results: list[SimplificationResult] = []
        for filepath in stl_files:
            results.append(self.simplify_if_needed(filepath, max_size_mb))

        return results


@dataclass
class CLIArgs:
    """Command-line arguments for mesh simplification."""

    directory: Annotated[Path, "Directory containing STL files (searched recursively)"]
    max_size: Annotated[float, "Max file size in MB"] = DEFAULT_MAX_SIZE_MB
    ratio: Annotated[
        float | None, "Direct reduction ratio (0.0-1.0), overrides max-size"
    ] = None
    quality: Annotated[float, "Quality threshold (0.0-1.0)"] = DEFAULT_QUALITY_THRESHOLD
    preserve_topology: Annotated[bool, "Preserve mesh topology"] = False
    preserve_boundary: Annotated[bool, "Preserve boundary edges"] = False


def main() -> None:
    """Command-line interface for mesh simplification."""
    import tyro

    args = tyro.cli(CLIArgs)

    if not args.directory.is_dir():
        log.error(f"Directory not found: {args.directory}")
        raise SystemExit(1)

    stl_files = _find_stl_files(args.directory)

    if not stl_files:
        log.warning(f"No STL files found in {args.directory}")
        raise SystemExit(0)

    log.info(f"Found {len(stl_files)} STL file(s) in {args.directory}")

    settings = DecimationSettings(
        quality_threshold=args.quality,
        preserve_topology=args.preserve_topology,
        preserve_boundary=args.preserve_boundary,
    )
    simplifier = MeshSimplifier(settings)

    for filepath in stl_files:
        if args.ratio is not None:
            log.info(f"Reducing {filepath.name} to {args.ratio * 100:.0f}% of faces")
            simplifier.reduce_faces(filepath, args.ratio)
            log.success(f"Done: {simplifier.get_file_size_mb(filepath):.2f}MB")
        else:
            simplifier.simplify_if_needed(filepath, args.max_size)


if __name__ == "__main__":
    main()
