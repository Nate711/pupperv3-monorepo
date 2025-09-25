"""Result processor for analyzing and visualizing benchmark results."""

from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import pandas as pd

from .bbox_types import BoundingBox
from .visualization import (
    create_comparison_grid,
    parse_bounding_boxes,
    save_individual_predictions,
)


class ResultProcessor:
    """Process and analyze benchmark results with bounding box support."""

    def __init__(self, results: List[Dict], image_dir: Path):
        """
        Initialize result processor.

        Args:
            results: List of benchmark result dictionaries
            image_dir: Directory containing original images
        """
        self.results = results
        self.image_dir = Path(image_dir)
        self.df = pd.DataFrame(results) if results else pd.DataFrame()

    def analyze_bounding_boxes(self) -> pd.DataFrame:
        """
        Analyze bounding box predictions across models.

        Returns:
            DataFrame with bounding box statistics per model/image
        """
        if self.df.empty:
            return pd.DataFrame()

        analysis_data = []

        for _, row in self.df.iterrows():
            if row['success'] and row['response']:
                boxes = parse_bounding_boxes(row['response'])
                analysis_data.append({
                    'model': row['model'],
                    'image': row['image'],
                    'num_boxes': len(boxes),
                    'boxes': boxes,
                    'latency': row['latency'],
                })

        analysis_df = pd.DataFrame(analysis_data)
        return analysis_df

    def generate_visualizations(
        self,
        output_dir: Optional[Path] = None,
        create_grids: bool = True,
        save_individual: bool = True,
    ) -> Dict[str, List[Path]]:
        """
        Generate visualization outputs for all results.

        Args:
            output_dir: Directory to save visualizations
            create_grids: Whether to create comparison grids
            save_individual: Whether to save individual annotated images

        Returns:
            Dictionary mapping visualization types to file paths
        """
        if output_dir is None:
            output_dir = Path.cwd()

        output_dir = Path(output_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        viz_dir = output_dir / f"run_{timestamp}"
        viz_dir.mkdir(parents=True, exist_ok=True)

        output_paths = {
            'grids': [],
            'individual': []
        }

        # Group results by image
        if not self.df.empty:
            grouped = self.df.groupby('image')

            for image_name, group_df in grouped:
                image_path = self.image_dir / image_name

                if not image_path.exists():
                    print(f"Warning: Image not found: {image_path}")
                    continue

                # Create comparison grid for this image
                if create_grids:
                    grid_path = viz_dir / f"grid_{image_path.stem}.jpg"
                    group_results = group_df.to_dict('records')
                    grid_img = create_comparison_grid(
                        image_path,
                        group_results,
                        grid_path
                    )
                    if grid_img:
                        output_paths['grids'].append(grid_path)

                # Save individual predictions
                if save_individual:
                    individual_dir = viz_dir / "individual"
                    individual_dir.mkdir(exist_ok=True)
                    group_results = group_df.to_dict('records')
                    saved_paths = save_individual_predictions(
                        image_path,
                        group_results,
                        individual_dir
                    )
                    output_paths['individual'].extend(saved_paths)

        return output_paths

    def print_bounding_box_summary(self):
        """Print summary statistics for bounding box predictions."""
        analysis_df = self.analyze_bounding_boxes()

        if analysis_df.empty:
            print("No bounding box data to analyze")
            return

        print("\n" + "=" * 60)
        print("BOUNDING BOX ANALYSIS")
        print("=" * 60)

        # Statistics by model
        model_stats = analysis_df.groupby('model')['num_boxes'].agg([
            'count', 'mean', 'std', 'min', 'max'
        ])

        print("\nBounding Boxes per Model:")
        print(model_stats.to_string())

        # Statistics by image
        image_stats = analysis_df.groupby('image')['num_boxes'].mean().sort_values()

        print("\nAverage Boxes Detected per Image:")
        for image, avg_boxes in image_stats.items():
            print(f"  {image}: {avg_boxes:.1f} boxes")

        # Model agreement analysis (for same images)
        print("\nModel Agreement Analysis:")
        for image_name in analysis_df['image'].unique():
            image_data = analysis_df[analysis_df['image'] == image_name]
            if len(image_data) > 1:
                box_counts = image_data['num_boxes'].values
                std_dev = box_counts.std()
                mean_count = box_counts.mean()
                print(f"  {image_name}:")
                print(f"    Mean boxes: {mean_count:.1f}")
                print(f"    Std deviation: {std_dev:.2f}")
                print(f"    Range: {box_counts.min()}-{box_counts.max()}")

    def export_detailed_results(
        self,
        output_dir: Optional[Path] = None,
        prefix: str = "detailed"
    ) -> Path:
        """
        Export detailed results including bounding box data.

        Args:
            output_dir: Directory to save results
            prefix: Prefix for output filename

        Returns:
            Path to saved file
        """
        if output_dir is None:
            output_dir = Path.cwd()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Prepare detailed data
        detailed_data = []
        for result in self.results:
            if result.get('success', False):
                boxes = parse_bounding_boxes(result.get('response', ''))
                detailed_data.append({
                    'model': result['model'],
                    'image': result['image'],
                    'latency': result['latency'],
                    'num_boxes': len(boxes),
                    'boxes': boxes,
                    'raw_response': result.get('response', ''),
                    'tokens_used': result.get('tokens_used'),
                })

        # Save to CSV (without box details)
        df_export = pd.DataFrame([
            {k: v for k, v in item.items() if k != 'boxes' and k != 'raw_response'}
            for item in detailed_data
        ])

        csv_path = output_dir / f"{prefix}_analysis_{timestamp}.csv"
        df_export.to_csv(csv_path, index=False)

        print(f"\nDetailed analysis saved to: {csv_path}")
        return csv_path