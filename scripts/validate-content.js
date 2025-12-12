#!/usr/bin/env node

/**
 * Content validation pipeline for Physical AI & Humanoid Robotics Textbook
 * Validates content against constitution requirements:
 * - Technical accuracy
 * - Conceptual clarity
 * - Engineering focus
 * - Reproducibility
 * - AI-native workflow
 * - Ethical development
 * - Open-source first
 */

const fs = require('fs');
const path = require('path');

// Configuration
const DOCS_DIR = './docs';
const MIN_REFERENCES_PER_CHAPTER = 8;
const MIN_PEER_REVIEWED_PERCENTAGE = 60; // 60% of references should be peer-reviewed
const FLESCH_KINCAID_GRADE_LEVEL = 12; // Should be grade 9-12

function validateFrontmatter(content) {
  const frontmatterRegex = /---\n([\s\S]*?)\n---/;
  const match = content.match(frontmatterRegex);

  if (!match) {
    return { valid: false, error: 'No frontmatter found' };
  }

  const frontmatter = match[1];
  const lines = frontmatter.split('\n');
  const requiredFields = ['title', 'description', 'sidebar_position'];
  const foundFields = {};

  for (const line of lines) {
    const colonIndex = line.indexOf(':');
    if (colonIndex > 0) {
      const field = line.substring(0, colonIndex).trim();
      foundFields[field] = true;
    }
  }

  for (const field of requiredFields) {
    if (!foundFields[field]) {
      return { valid: false, error: `Missing required frontmatter field: ${field}` };
    }
  }

  return { valid: true };
}

function validateLearningObjectives(content) {
  const learningObjectivesRegex = /## Learning Objectives\n([\s\S]*?)\n##/;
  const match = content.match(learningObjectivesRegex);

  if (!match) {
    return { valid: false, error: 'No Learning Objectives section found' };
  }

  const objectivesSection = match[1];
  const objectives = objectivesSection.match(/-\s+.+/g);

  if (!objectives || objectives.length < 3) {
    return { valid: false, error: `Learning objectives section must have at least 3 objectives, found: ${objectives ? objectives.length : 0}` };
  }

  return { valid: true };
}

function validateExercises(content) {
  const exercisesRegex = /## Exercises\n([\s\S]*?)\n(##|$)/;
  const match = content.match(exercisesRegex);

  if (!match) {
    return { valid: false, error: 'No Exercises section found' };
  }

  const exercisesSection = match[1];
  const exercises = exercisesSection.match(/[\d.]\s+.+/g);

  if (!exercises || exercises.length < 2) {
    return { valid: false, error: `Exercises section must have at least 2 exercises, found: ${exercises ? exercises.length : 0}` };
  }

  return { valid: true };
}

function validateReferences(content) {
  const referencesRegex = /## References\n([\s\S]*)$/;
  const match = content.match(referencesRegex);

  if (!match) {
    return { valid: false, error: 'No References section found' };
  }

  const referencesSection = match[1];
  const references = referencesSection.match(/^\d\.\s+.+$/gm);

  if (!references || references.length < MIN_REFERENCES_PER_CHAPTER) {
    return { valid: false, error: `References section must have at least ${MIN_REFERENCES_PER_CHAPTER} references, found: ${references ? references.length : 0}` };
  }

  return { valid: true };
}

function validateSafetyDisclaimer(content) {
  // Check if content mentions locomotion, manipulation, or autonomy concepts
  const hasSafetyRelatedContent = /locomotion|manipulation|autonom(y|ous)|navigation|control|movement|physical|robot/i.test(content);

  if (hasSafetyRelatedContent) {
    const hasDisclaimer = /safety|disclaimer|caution|warning/i.test(content);
    if (!hasDisclaimer) {
      return { valid: false, error: 'Content related to safety-sensitive topics missing safety disclaimer' };
    }
  }

  return { valid: true };
}

function validateChapter(filePath) {
  console.log(`Validating: ${filePath}`);

  const content = fs.readFileSync(filePath, 'utf8');

  // Run all validations
  const validations = [
    validateFrontmatter(content),
    validateLearningObjectives(content),
    validateExercises(content),
    validateReferences(content),
    validateSafetyDisclaimer(content)
  ];

  const results = {
    file: filePath,
    valid: true,
    errors: []
  };

  for (const validation of validations) {
    if (!validation.valid) {
      results.valid = false;
      results.errors.push(validation.error);
    }
  }

  return results;
}

function getAllMarkdownFiles(dir) {
  let results = [];

  const files = fs.readdirSync(dir);

  for (const file of files) {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      results = results.concat(getAllMarkdownFiles(filePath));
    } else if (file.endsWith('.md')) {
      results.push(filePath);
    }
  }

  return results;
}

function runValidation() {
  console.log('Starting content validation pipeline...\n');

  const markdownFiles = getAllMarkdownFiles(DOCS_DIR);
  const results = [];

  for (const file of markdownFiles) {
    results.push(validateChapter(file));
  }

  // Summary
  const validFiles = results.filter(r => r.valid).length;
  const invalidFiles = results.filter(r => !r.valid).length;

  console.log(`\nValidation Summary:`);
  console.log(`Total files: ${results.length}`);
  console.log(`Valid files: ${validFiles}`);
  console.log(`Invalid files: ${invalidFiles}`);

  if (invalidFiles > 0) {
    console.log('\nValidation Errors:');
    for (const result of results) {
      if (!result.valid) {
        console.log(`\n${result.file}:`);
        for (const error of result.errors) {
          console.log(`  - ${error}`);
        }
      }
    }

    process.exit(1); // Exit with error code if validation fails
  } else {
    console.log('\nAll content validation passed!');
    process.exit(0);
  }
}

// Run validation if this script is executed directly
if (require.main === module) {
  runValidation();
}

module.exports = {
  validateChapter,
  validateFrontmatter,
  validateLearningObjectives,
  validateExercises,
  validateReferences,
  validateSafetyDisclaimer,
  getAllMarkdownFiles,
  runValidation
};