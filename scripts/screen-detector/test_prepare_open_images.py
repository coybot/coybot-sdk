import hashlib
import importlib.util
import pathlib
import tempfile
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("prepare_open_images.py")
SPEC = importlib.util.spec_from_file_location("prepare_open_images", MODULE_PATH)
prepare = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare)


class PrepareOpenImagesTests(unittest.TestCase):
    def test_converts_normalized_box_to_create_ml_pixel_center_box(self):
        row = {
            "XMin": "0.10",
            "XMax": "0.50",
            "YMin": "0.20",
            "YMax": "0.60",
        }

        self.assertEqual(
            prepare.create_ml_coordinates(row, width=1000, height=500),
            {"x": 300.0, "y": 200.0, "width": 400.0, "height": 200.0},
        )

    def test_build_examples_collapses_positive_classes_and_keeps_clean_negatives(self):
        rows = [
            annotation("positive", "/m/02522", xmin="0.10", xmax="0.50"),
            annotation("positive", "/m/07c52", xmin="0.55", xmax="0.95"),
            annotation("negative", "/m/01n5jq"),
            annotation("mixed", "/m/0d4v4"),
            annotation("mixed", "/m/02522"),
            annotation("ignored", "/m/01pns0"),
        ]
        dimensions = {
            image_id: (1000, 500)
            for image_id in ["positive", "negative", "mixed", "ignored"]
        }

        examples = prepare.build_examples(rows, dimensions)

        self.assertEqual(
            [item["imagefilename"] for item in examples],
            ["mixed.jpg", "negative.jpg", "positive.jpg"],
        )
        self.assertEqual([a["label"] for a in examples[2]["annotation"]], ["screen", "screen"])
        self.assertEqual(examples[1]["annotation"], [])
        self.assertEqual([a["label"] for a in examples[0]["annotation"]], ["screen"])

    def test_partition_is_deterministic_disjoint_and_complete(self):
        ids = [f"image-{index:03d}" for index in range(20)]

        first = prepare.partition_image_ids(ids, seed=42, validation_fraction=0.2, test_fraction=0.2)
        second = prepare.partition_image_ids(reversed(ids), seed=42, validation_fraction=0.2, test_fraction=0.2)

        self.assertEqual(first, second)
        train, validation, test = map(set, first)
        self.assertFalse(train & validation)
        self.assertFalse(train & test)
        self.assertFalse(validation & test)
        self.assertEqual(train | validation | test, set(ids))

    def test_partition_manifests_are_honored_and_must_be_disjoint(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            paths = {}
            for split, values in {
                "train": ["image-a", "image-b"],
                "validation": ["image-c"],
                "test": ["image-d"],
            }.items():
                path = root / f"{split}.txt"
                path.write_text("\n".join(values) + "\n", encoding="utf-8")
                paths[split] = path

            self.assertEqual(
                prepare.read_partition_manifests(paths, {"image-a", "image-b", "image-c", "image-d"}),
                (["image-a", "image-b"], ["image-c"], ["image-d"]),
            )

            paths["test"].write_text("image-c\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "overlap"):
                prepare.read_partition_manifests(paths, {"image-a", "image-b", "image-c", "image-d"})

    def test_sha256_file_returns_content_digest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.bin"
            path.write_bytes(b"phrover-screen")

            self.assertEqual(
                prepare.sha256_file(path),
                hashlib.sha256(b"phrover-screen").hexdigest(),
            )


def annotation(image_id, label, xmin="0.10", xmax="0.50"):
    return {
        "ImageID": image_id,
        "LabelName": label,
        "XMin": xmin,
        "XMax": xmax,
        "YMin": "0.20",
        "YMax": "0.60",
    }


if __name__ == "__main__":
    unittest.main()
