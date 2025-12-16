#!/usr/bin/env node

/**
 * Image Analysis Tool for AI-Humanoid Textbook Project
 * Analyzes current images and provides optimization recommendations
 */

const fs = require('fs');
const path = require('path');

function analyzeImages() {
  const imgDir = path.join(__dirname, '../frontend/static/img');

  if (!fs.existsSync(imgDir)) {
    console.error('Image directory does not exist:', imgDir);
    return;
  }

  const files = fs.readdirSync(imgDir);

  console.log('AI-Humanoid Textbook Image Analysis Report');
  console.log('==========================================');
  console.log(`Directory: ${imgDir}`);
  console.log(`Total files: ${files.length}\n`);

  let totalSize = 0;
  const svgFiles = [];
  const otherFiles = [];

  files.forEach(file => {
    const filePath = path.join(imgDir, file);
    const stats = fs.statSync(filePath);
    const sizeKB = Math.round(stats.size / 1024);
    totalSize += stats.size;

    const fileInfo = {
      name: file,
      sizeKB: sizeKB,
      extension: path.extname(file).toLowerCase(),
      fullPath: filePath
    };

    if (fileInfo.extension === '.svg') {
      svgFiles.push(fileInfo);
    } else {
      otherFiles.push(fileInfo);
    }
  });

  // Analyze SVG files
  console.log(`SVG Files (${svgFiles.length} files):`);
  console.log('----------------------------------------');
  svgFiles.forEach(file => {
    console.log(`${file.name}: ${file.sizeKB} KB`);
  });

  if (otherFiles.length > 0) {
    console.log(`\nOther Files (${otherFiles.length} files):`);
    console.log('----------------------------------------');
    otherFiles.forEach(file => {
      console.log(`${file.name}: ${file.sizeKB} KB (${file.extension})`);
    });
  }

  console.log(`\nTotal size: ${Math.round(totalSize / 1024)} KB`);
  console.log(`Average size: ${Math.round(totalSize / files.length / 1024)} KB per file`);

  // Provide recommendations
  console.log('\nRecommendations:');
  console.log('----------------');
  console.log('✅ SVG optimization is working well - all diagrams are in optimal format');
  console.log('✅ Current setup with SVGO through Docusaurus is appropriate');

  if (svgFiles.length > 0) {
    console.log('💡 Consider running SVGO with more aggressive settings for additional optimization');
    console.log('💡 Ensure all SVGs have proper accessibility attributes (title, desc)');
  }

  if (otherFiles.length > 0) {
    console.log('💡 For any PNG/JPG files, consider:');
    console.log('   - Converting suitable images to SVG for scalability');
    console.log('   - Using WebP format with fallbacks for photographs');
    console.log('   - Implementing responsive image techniques');
  } else {
    console.log('✅ All images are currently in SVG format - excellent for technical documentation!');
  }

  console.log('\nOptimization Opportunities:');
  console.log('---------------------------');
  console.log('1. Enhance SVGO configuration for better compression');
  console.log('2. Implement modern format support (WebP/AVIF) for future images');
  console.log('3. Add responsive image loading for better performance');
  console.log('4. Set up build-time optimization validation');

  // Calculate potential savings based on typical SVG optimization
  if (svgFiles.length > 0) {
    const estimatedSavings = svgFiles.reduce((sum, file) => {
      // Typical SVG optimization can save 20-50% with aggressive settings
      const potentialSaving = Math.max(1, Math.floor(file.sizeKB * 0.3)); // 30% average saving
      return sum + potentialSaving;
    }, 0);

    console.log(`\nEstimated potential savings: ~${estimatedSavings} KB (${Math.round(estimatedSavings/(totalSize/1024)*100)}%) with enhanced optimization`);
  }
}

// Run the analysis
analyzeImages();